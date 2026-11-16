#![no_std]
#![no_main]
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    time::{Duration, Instant},
};
use esp_hal::{
    gpio::DriveMode,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    time::Rate,
};
use log::{info, warn};
esp_bootloader_esp_idf::esp_app_desc!();

// Four-wire SPI mode 0, MSB first. Dedicated GPIO ownership; no shared bus.
struct Oled {
    clk: Output<'static>,
    mosi: Output<'static>,
    cs: Output<'static>,
    dc: Output<'static>,
}
impl Oled {
    fn send(&mut self, data: bool, bytes: &[u8]) {
        self.dc
            .set_level(if data { Level::High } else { Level::Low });
        self.cs.set_low();
        let delay = Delay::new();
        for byte in bytes {
            for bit in (0..8).rev() {
                self.mosi.set_level(if byte & (1 << bit) != 0 {
                    Level::High
                } else {
                    Level::Low
                });
                delay.delay_micros(1);
                self.clk.set_high();
                delay.delay_micros(1);
                self.clk.set_low();
            }
        }
        self.cs.set_high();
    }
    fn eyes(&mut self) {
        self.send(
            false,
            &[
                0xae, 0xd5, 0x80, 0xa8, 63, 0xd3, 0, 0x40, 0x8d, 0x14, 0x20, 0, 0xa1, 0xc8, 0xda,
                0x12, 0x81, 0x7f, 0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6,
            ],
        );
        self.send(false, &[0x21, 0, 127, 0x22, 0, 7]);
        self.send(false, &[0xaf]);
    }
    fn render(&mut self, pose: esp_bimo::eyes::Pose) {
        self.send(false, &[0x21, 0, 127, 0x22, 0, 7]);
        self.send(true, &esp_bimo::eyes::render(pose));
    }
}

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());
    let _stby = Output::new(p.GPIO40, Level::Low, OutputConfig::default());
    macro_rules! low {
        ($pin:expr) => {
            Output::new($pin, Level::Low, OutputConfig::default())
        };
    }
    let _motor_pins = (
        low!(p.GPIO5),
        low!(p.GPIO6),
        low!(p.GPIO7),
        low!(p.GPIO15),
        low!(p.GPIO16),
        low!(p.GPIO17),
    );
    // GPIO18/48 remain untouched. Do not construct Board or a motor/servo driver.
    esp_println::logger::init_logger_from_env();
    // Existing board wiring: active-low buzzer on GPIO21. PWM runs in hardware.
    let buzzer_pin = Output::new(p.GPIO21, Level::High, OutputConfig::default());
    let mut ledc = Ledc::new(p.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut tones = [
        ledc.timer::<LowSpeed>(timer::Number::Timer0),
        ledc.timer::<LowSpeed>(timer::Number::Timer1),
        ledc.timer::<LowSpeed>(timer::Number::Timer2),
        ledc.timer::<LowSpeed>(timer::Number::Timer3),
    ];
    for (timer, hz) in tones.iter_mut().zip([880, 1320, 1760, 2640]) {
        timer
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty8Bit,
                clock_source: timer::LSClockSource::APBClk,
                frequency: Rate::from_hz(hz),
            })
            .expect("tickle tone");
    }
    let mut buzzer = ledc.channel(
        channel::Number::Channel0,
        buzzer_pin
            .into_peripheral_output()
            .with_output_inverter(true),
    );
    buzzer
        .configure(channel::config::Config {
            timer: &tones[0],
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .expect("silent buzzer");
    // Chirping giggle: short rising bursts separated by rests; latest touch restarts.
    const REST: usize = usize::MAX;
    const GIGGLE: [(usize, u64); 12] = [
        (0, 65),
        (1, 55),
        (2, 70),
        (REST, 65),
        (1, 55),
        (2, 60),
        (0, 70),
        (REST, 75),
        (0, 50),
        (2, 65),
        (1, 80),
        (REST, 40),
    ];
    // Low duty limits excitation for a gentle, short appreciative chirp.
    // Actual loudness depends on the buzzer and must be checked on hardware.
    const CONTENT: [(usize, u64); 12] = [
        (0, 45),
        (0, 55),
        (REST, 25),
        (1, 65),
        (1, 70),
        (REST, 70),
        (2, 45),
        (2, 55),
        (REST, 25),
        (3, 65),
        (3, 70),
        (REST, 1),
    ];
    const LIFTED: [(usize, u64); 10] = [
        (0, 90),
        (3, 45),
        (1, 80),
        (REST, 35),
        (2, 50),
        (0, 110),
        (3, 60),
        (1, 100),
        (0, 140),
        (REST, 1),
    ];
    // Saturation is a hardware-test heuristic, not proof of being lifted.
    let mut ir_latched = false;
    let mut ir_high_count = 0u8;
    let mut ir_low_count = 0u8;
    let mut sound: &[(usize, u64)] = &GIGGLE;
    let mut gentle = false;
    let mut sound_step = sound.len();
    let mut sound_due = 0u64;
    let delay = Delay::new();
    let mut reset = low!(p.GPIO42);
    delay.delay_millis(10);
    reset.set_high();
    delay.delay_millis(100);
    let mut oled = Oled {
        clk: low!(p.GPIO8),
        mosi: low!(p.GPIO9),
        cs: Output::new(p.GPIO10, Level::High, OutputConfig::default()),
        dc: low!(p.GPIO11),
    };
    oled.eyes();
    let head = Input::new(p.GPIO43, InputConfig::default());
    let chest = Input::new(p.GPIO44, InputConfig::default());
    let tilt = Input::new(p.GPIO39, InputConfig::default());
    let mut trig = low!(p.GPIO38);
    let echo = Input::new(p.GPIO47, InputConfig::default());
    let mut cfg = AdcConfig::new();
    let mut mic = cfg.enable_pin(p.GPIO2, Attenuation::_11dB);
    let mut ir = cfg.enable_pin(p.GPIO1, Attenuation::_11dB);
    let mut adc = Adc::new(p.ADC1, cfg);
    info!(
        "Hardware diagnostics: STBY LOW; motors DISARMED; servo unused; USB Serial/JTAG GPIO19/20 only"
    );
    let mut previous = (head.is_high(), chest.is_high(), tilt.is_high());
    info!("initial head/chest/tilt raw={previous:?}");
    let mut eyes = esp_bimo::eyes::Eyes::new(Instant::now().duration_since_epoch().as_millis());
    let mut head_db = esp_bimo::eyes::Debounce::new(previous.0);
    let mut chest_db = esp_bimo::eyes::Debounce::new(previous.1);
    let mut rendered = None;
    // No OLED transfer or ADC/logging during echo acquisition: preserve pulse timing.
    let mut ping_due = Instant::now();
    let mut echo_phase = 0u8;
    let mut echo_start = Instant::now();
    loop {
        let start = Instant::now();
        let mut min = 4095u16;
        let mut max = 0u16;
        let mut sum = 0u32;
        let mut rails = 0;
        let mut count = 0u32;
        while start.elapsed() < Duration::from_millis(250) {
            let now = Instant::now();
            if echo_phase != 0 {
                if echo_phase == 1 && echo.is_high() {
                    echo_phase = 2;
                    echo_start = now;
                } else if echo_phase == 2 && echo.is_low() {
                    let us = echo_start.elapsed().as_micros();
                    echo_phase = 0;
                    info!("HC-SR04 echo_us={us} distance_mm={}", us * 343 / 2000);
                } else if echo_start.elapsed() >= Duration::from_millis(30) {
                    echo_phase = 0;
                    warn!("HC-SR04 timeout");
                }
                // Timeouts are polled, never waited on. Continue servicing eyes below.
            } else if now >= ping_due {
                ping_due = now + Duration::from_millis(300);
                if echo.is_high() {
                    warn!("HC-SR04 echo already HIGH");
                } else {
                    trig.set_high();
                    delay.delay_micros(10);
                    trig.set_low();
                    echo_start = Instant::now();
                    echo_phase = 1;
                }
            }
            let ms = now.duration_since_epoch().as_millis();
            if let Some(high) = head_db.update(head.is_high(), ms) {
                info!("touch head debounced={high}");
                if high {
                    eyes.trigger(esp_bimo::eyes::Event::Head, ms);
                    sound = &CONTENT;
                    gentle = true;
                    sound_step = 0;
                    sound_due = ms;
                }
            }
            if let Some(high) = chest_db.update(chest.is_high(), ms) {
                info!("touch chest debounced={high}");
                if high {
                    eyes.trigger(esp_bimo::eyes::Event::Chest, ms);
                    sound = &GIGGLE;
                    gentle = false;
                    sound_step = 0;
                    sound_due = ms;
                }
            }
            if sound_step < sound.len() && ms >= sound_due {
                let (tone, duration) = sound[sound_step];
                if tone == REST {
                    buzzer.set_duty(0).expect("tickle rest");
                } else {
                    buzzer
                        .configure(channel::config::Config {
                            timer: &tones[tone],
                            duty_pct: if gentle {
                                // Gentle attack followed by a softer tail on each note.
                                if sound_step % 3 == 0 { 5 } else { 2 }
                            } else {
                                50
                            },
                            drive_mode: DriveMode::PushPull,
                        })
                        .expect("tickle note");
                }
                sound_step += 1;
                sound_due = ms + duration;
                // Last sequence entry is a rest, leaving GPIO21 HIGH indefinitely.
            }
            let pose = eyes.update(ms);
            if echo_phase != 0 {
                continue;
            }
            if rendered != Some(pose) {
                oled.render(pose);
                rendered = Some(pose);
            }
            let value = adc.read_blocking(&mut mic);
            min = min.min(value);
            max = max.max(value);
            sum += value as u32;
            count += 1;
            if value <= 16 || value >= 4079 {
                rails += 1;
            }
            let states = (head.is_high(), chest.is_high(), tilt.is_high());
            if states.2 != previous.2 {
                info!("tilt raw={}", states.2);
            }
            previous = states;
            delay.delay_micros(250);
        }
        let ir_raw = adc.read_blocking(&mut ir);
        if ir_raw == 4095 {
            ir_high_count = ir_high_count.saturating_add(1);
            ir_low_count = 0;
        } else {
            ir_high_count = 0;
            ir_low_count = if ir_raw < 3900 {
                ir_low_count.saturating_add(1)
            } else {
                0
            };
        }
        if !ir_latched && ir_high_count >= 2 {
            ir_latched = true;
            sound = &LIFTED;
            gentle = false;
            sound_step = 0;
            sound_due = Instant::now().duration_since_epoch().as_millis();
            eyes.trigger(esp_bimo::eyes::Event::Lifted, sound_due);
            warn!(
                "IR saturation: possible lifted; surprised eye wobble and unusual sound triggered; servo unused"
            );
        }
        if ir_latched && ir_low_count >= 2 {
            ir_latched = false;
            info!("IR below 3900: lift detection rearmed");
        }
        info!(
            "MIC window_ms={} n={count} raw_mean={} min={min} max={max} peak_to_peak={} near_rails={rails}; IR raw={}",
            start.elapsed().as_millis(),
            sum / count.max(1),
            max - min,
            ir_raw
        );
    }
}
