//! Module providing a global API for controlling GPIO pins.

use core::cell::RefCell;
use embassy_sync::{blocking_mutex::CriticalSectionMutex, once_lock::OnceLock};
use super::GpiosResources;
use embassy_stm32::{
    Peri,
    gpio::{AnyPin, Level, Speed, Output, Input, Pull},
};

pub struct OutputPin {
    pin: CriticalSectionMutex<RefCell<Output<'static>>>,
}
impl OutputPin {
    fn new(pin: Peri<'static, AnyPin>, initial_output: Level, speed: Speed) -> Self {
        Self {
            pin: CriticalSectionMutex::new(RefCell::new(Output::new(pin, initial_output, speed))),
        }
    }

    /// Set the output as high.
    pub fn set_high(&self) {
        self.pin.lock(|i|
            // PANIC SAFETY: The RefCell is only ever borrowed (mutably or non-mutably) in the closures implemented on OutputPin's wrappers,
            // so there will be no borrows live at this moment.
            i.borrow_mut().set_high())
    }

    /// Set the output as low.
    pub fn set_low(&self) {
        self.pin.lock(|i|
            // PANIC SAFETY: The RefCell is only ever borrowed (mutably or non-mutably) in the closures implemented on OutputPin's wrappers,
            // so there will be no borrows live at this moment.
            i.borrow_mut().set_low())
    }

    /// Set the output level.
    pub fn set_level(&self, level: Level) {
        self.pin.lock(|i|
            // PANIC SAFETY: The RefCell is only ever borrowed (mutably or non-mutably) in the closures implemented on OutputPin's wrappers,
            // so there will be no borrows live at this moment.
            i.borrow_mut().set_level(level))
    }

    /// Is the output pin set as high?
    pub fn is_set_high(&self) -> bool {
        self.pin.lock(|i|
            // PANIC SAFETY: The RefCell is only ever borrowed (mutably or non-mutably) in the closures implemented on OutputPin's wrappers,
            // so there will be no borrows live at this moment.
            i.borrow_mut().is_set_high())
    }

    /// Is the output pin set as low?
    pub fn is_set_low(&self) -> bool {
        self.pin.lock(|i|
            // PANIC SAFETY: The RefCell is only ever borrowed (mutably or non-mutably) in the closures implemented on OutputPin's wrappers,
            // so there will be no borrows live at this moment.
            i.borrow_mut().is_set_low())
    }

    /// What level the output is set to
    pub fn output_level(&self) -> Level {
        self.pin.lock(|i|
            // PANIC SAFETY: The RefCell is only ever borrowed (mutably or non-mutably) in the closures implemented on OutputPin's wrappers,
            // so there will be no borrows live at this moment.
            i.borrow_mut().output_level())
    }

    /// Toggle pin output
    pub fn toggle(&self) {
        self.pin.lock(|i|
            // PANIC SAFETY: The RefCell is only ever borrowed (mutably or non-mutably) in the closures implemented on OutputPin's wrappers,
            // so there will be no borrows live at this moment.
            i.borrow_mut().toggle())
    }
}

pub struct InputPin {
    pin: CriticalSectionMutex<RefCell<Input<'static>>>,
}
impl InputPin {
    fn new(pin: Peri<'static, AnyPin>, pull: Pull) -> Self {
        Self { pin: CriticalSectionMutex::new(RefCell::new(Input::new(pin, pull))) }
    }

    /// Get whether the pin input level is high.
    pub fn is_high(&self) -> bool {
        self.pin.lock(|i|
            // PANIC SAFETY: The RefCell is only ever borrowed (mutably or non-mutably) in the closures implemented on InputPin's wrappers,
            // so there will be no borrows live at this moment.
            i.borrow().is_high())
    }

    /// Get whether the pin input level is low.
    pub fn is_low(&self) -> bool {
        self.pin.lock(|i|
            // PANIC SAFETY: The RefCell is only ever borrowed (mutably or non-mutably) in the closures implemented on InputPin's wrappers,
            // so there will be no borrows live at this moment.
            i.borrow().is_low())
    }

    /// Get the current pin input level.
    pub fn level(&self) -> Level {
        self.pin.lock(|i|
            // PANIC SAFETY: The RefCell is only ever borrowed (mutably or non-mutably) in the closures implemented on InputPin's wrappers,
            // so there will be no borrows live at this moment.
            i.borrow().level())
    }
}

pub struct Pins {
    pub mcu_fault_pin: OutputPin,
    pub ts_minus_sense_pin: InputPin,
    pub ts_plus_sense_pin: InputPin,
    pub acc_sense_pin: InputPin,
    pub tsip_sense_pin: InputPin,
}

pub struct Handle {
    pins: OnceLock<Pins>,
}
impl Handle {
    const fn new() -> Self {
        Handle { pins: OnceLock::new() }
    }

    /// Helper to once-initialize the pins.
    ///
    /// ### Panics
    /// Will panic if we try to call this more than once.
    fn init(&self, value: Pins) {
        self.pins.init(value).unwrap_or_else(|_| panic!("Error calling `Handle::init()`. We tried to initialize `Handle` more than once for some reason."))
    }

    /// Returns a reference to the pins.
    ///
    /// ###
    /// Will panic if this is called before the pins have been initialized. The pins get initialized very early in the program (i.e., before tasks and such should
    /// be spawned), so it should be fine to call this from most places without worrying about it. However, if you're in an ISR or some other context where it could
    /// technically be possible for the initialization to not have happened yet, you can look at `try_get()`.
    pub fn get(&self) -> &Pins {
        self.pins.try_get().expect("We tried to access GPIO pins using `Handle::get()` before they have been initialized. Weird! They should be initialized very early on.")
    }

    /// Attempts to get a reference to the pins. If the pins have not been initialized yet, returns `None`.
    pub fn try_get(&self) -> Option<&Pins> {
        self.pins.try_get()
    }
}

static PINS: Handle = Handle::new();

/// Provides access to the pins handle.
pub fn pins() -> &'static Handle {
    &PINS
}

/// Initializes the GPIOs relavent to this module.
///
/// Obviously, not all GPIOs in the whole entire project are intialized here (since many pins are passed to specific peripherals or other subsystems).
/// This is just for the basic GPIOs we decide to explicitly control here.
pub fn init(r: GpiosResources) {
    PINS.init(Pins {
        mcu_fault_pin: OutputPin::new(r.mcu_fault_pin.into(), Level::Low, Speed::Low),

        // The shutdown circuit drives these lines actively, so no internal pull.
        ts_minus_sense_pin: InputPin::new(r.ts_minus_sense_pin.into(), Pull::None),
        ts_plus_sense_pin: InputPin::new(r.ts_plus_sense_pin.into(), Pull::None),
        acc_sense_pin: InputPin::new(r.acc_sense_pin.into(), Pull::None),
        tsip_sense_pin: InputPin::new(r.tsip_sense_pin.into(), Pull::None),
    })
}
