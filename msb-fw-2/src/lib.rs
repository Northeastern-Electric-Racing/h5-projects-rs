#![no_std]
pub mod analog_sensor;
pub mod bind_interrupts;
pub mod can;
pub mod multiplexor_handler;
pub mod shock_pot;
pub mod steering_angle;
pub mod strain_guage;
pub mod wheel_speed;
// TODO: re-enable once the lis2mdl-ner dependency resolves (see Cargo.toml).
// pub mod magnetometer;
pub mod hdc2021;
pub mod load_cell;
pub mod thermocouple;
// TODO: re-enable once the lsm6dsv-ner IMU driver exists.
// pub mod imu;
