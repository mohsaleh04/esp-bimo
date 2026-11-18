use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output};

pub struct Oled {
    pub clk: Output<'static>,
    pub mosi: Output<'static>,
    pub cs: Output<'static>,
    pub dc: Output<'static>,
}

impl Oled {
    pub fn send(&mut self, data: bool, bytes: &[u8]) {
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
    pub fn eyes(&mut self) {
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
    pub fn render(&mut self, pose: crate::eyes::Pose) {
        self.send(false, &[0x21, 0, 127, 0x22, 0, 7]);
        self.send(true, &crate::eyes::render(pose));
    }
}
