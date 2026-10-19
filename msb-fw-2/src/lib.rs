#![no_std]
pub mod analog_sensor;
pub mod bind_interrupts;
<<<<<<< Updated upstream
pub mod can;
=======
pub mod load_cell;
pub mod magnetometer;
>>>>>>> Stashed changes
pub mod multiplexor_handler;
pub mod shock_pot;
pub mod steering_angle;
pub mod strain_guage;
pub mod thermocouple;
pub mod wheel_speed;
// pub mod imu;
pub mod hdc2021;
<<<<<<< Updated upstream
// TODO: re-enable once the lis2mdl-ner dependency resolves (see Cargo.toml).
// pub mod magnetometer;
pub mod hdc2021;
pub mod load_cell;
pub mod thermocouple;
// TODO: re-enable once the lsm6dsv-ner IMU driver exists.
// pub mod imu;
=======
>>>>>>> Stashed changes
