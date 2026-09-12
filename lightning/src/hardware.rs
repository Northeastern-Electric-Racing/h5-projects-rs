#![no_std]
#![no_main]

// TODO: Add LED I/O and (later) lightning sensor I/O
mod hardware {
    pub fn set_all_off();
    pub fn set_red_on();
    pub fn set_green_on();
}
