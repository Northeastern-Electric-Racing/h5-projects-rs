pub use self::hardware::Leds;

// TODO: Add lightning sensor I/O here
mod hardware {
    use embassy_stm32::Peri;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_stm32::peripherals::{PE4, PE5};

    /// Owns the two status LEDs
    pub struct Leds {
        red: Output<'static>,
        green: Output<'static>,
    }

    impl Leds {
        pub fn new(red: Peri<'static, PE4>, green: Peri<'static, PE5>) -> Self {
            Self {
                red: Output::new(red, Level::Low, Speed::Low),
                green: Output::new(green, Level::Low, Speed::Low),
            }
        }

        pub fn set_all_off(&mut self) {
            self.red.set_low();
            self.green.set_low();
        }

        pub fn set_red_on(&mut self) {
            self.red.set_high();
            self.green.set_low();
        }

        pub fn set_green_on(&mut self) {
            self.red.set_low();
            self.green.set_high();
        }
    }
}
