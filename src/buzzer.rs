//! A continuously repeating melody for the active-low buzzer input on GPIO21.
use embassy_time::Timer;
use esp_hal::{
    gpio::{DriveMode, Output},
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::LEDC,
    time::Rate,
};

#[embassy_executor::task]
pub async fn play(peripheral: LEDC<'static>, pin: Output<'static>) {
    let mut ledc = Ledc::new(peripheral);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // A4, C5, E5 and G5: notes from A natural minor.
    let mut timers = [
        ledc.timer::<LowSpeed>(timer::Number::Timer0),
        ledc.timer::<LowSpeed>(timer::Number::Timer1),
        ledc.timer::<LowSpeed>(timer::Number::Timer2),
        ledc.timer::<LowSpeed>(timer::Number::Timer3),
    ];
    for (timer, frequency) in timers.iter_mut().zip([440, 523, 659, 784]) {
        timer
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty8Bit,
                clock_source: timer::LSClockSource::APBClk,
                frequency: Rate::from_hz(frequency),
            })
            .expect("buzzer timer configuration");
    }

    // Invert the physical signal: logical 0% duty becomes HIGH (silent).
    let output = pin.into_peripheral_output().with_output_inverter(true);
    let mut channel = ledc.channel(channel::Number::Channel0, output);
    // Twelve bars in A minor, with a recurring motif and a contrasting middle.
    // Durations are eighth-note ticks at about 167 BPM (180 ms per tick).
    const A: usize = 0;
    const C: usize = 1;
    const E: usize = 2;
    const G: usize = 3;
    const REST: usize = 4;
    let melody = [
        // Opening: introduce the motif, then answer it.
        (A, 1),
        (C, 1),
        (E, 2),
        (C, 1),
        (A, 1),
        (C, 2),
        (E, 1),
        (G, 1),
        (E, 2),
        (C, 2),
        (A, 2),
        (A, 1),
        (C, 1),
        (E, 1),
        (G, 1),
        (E, 2),
        (C, 2),
        (E, 1),
        (C, 1),
        (A, 4),
        (REST, 2),
        // Middle: climb to G, with shorter notes and longer answering tones.
        (C, 1),
        (E, 1),
        (G, 2),
        (G, 1),
        (E, 1),
        (C, 2),
        (E, 1),
        (G, 1),
        (E, 1),
        (C, 1),
        (E, 3),
        (REST, 1),
        (G, 2),
        (E, 1),
        (G, 1),
        (E, 2),
        (C, 2),
        (E, 1),
        (C, 1),
        (A, 2),
        (C, 1),
        (E, 1),
        (G, 2),
        // Return: vary the opening, then settle on A before repeating.
        (A, 1),
        (C, 1),
        (E, 2),
        (G, 1),
        (E, 1),
        (C, 2),
        (E, 1),
        (G, 1),
        (E, 2),
        (C, 1),
        (A, 1),
        (C, 2),
        (G, 1),
        (E, 1),
        (C, 2),
        (E, 1),
        (C, 1),
        (A, 2),
        (C, 1),
        (E, 1),
        (A, 4),
        (REST, 2),
    ];
    for (note, ticks) in melody {
        let duration_ms = ticks * 180;
        if note == REST {
            Timer::after_millis(duration_ms).await;
            continue;
        }
        let timer = &timers[note];
        channel
            .configure(channel::config::Config {
                timer,
                duty_pct: 50,
                drive_mode: DriveMode::PushPull,
            })
            .expect("buzzer channel configuration");
        Timer::after_millis(duration_ms - 25).await;
        channel.set_duty(0).expect("buzzer silence");
        Timer::after_millis(25).await;
    }
}
