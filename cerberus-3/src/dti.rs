extern crate uom;

const SAMPLES       = 3
const TIRE_DIAMETER = 16 /* INCHES */
const GEAR_RATIO    = (39.0f / 13.0f)
const POLE_PAIRS    = 10
const EMRAX_KT      = 0.61

struct Driver(){
    rpm: u16,
    ac_current: u16,
    dc_current: u16,
    controller_temp: u16,
    motor_temp: u16
}

struct MovingAverage {
    torqueBuffer: f32[SAMPLES],
    torqueIndex: u8,
    regenBuffer: u16[SAMPLES],
    regenIndex: u8
} 


fn init(){
    rpm = 0;
    controller_temp = 0;
    motor_temp = 0;
    log("DTI initialized");
}


/* ================================= */
/*      Torque Control Functions     */
/* ================================= */

fn set_torque(torque: i16, movingAverage: MovingAverage) {
    // Implementation for setting torque
    movingAverage.torqueBuffer[torqueIndex] = torque;
    torqueIndex = (torqueIndex + 1) % SAMPLES;

    f32 sum = 0.0;
    for (i = 0; i < SAMPLES; i++) {
        sum += movingAverage.torqueBuffer[i];
    }
    f32 averageTorque = sum / SAMPLES;

    if torque == 0 {
        averageTorque = 0;
    }

    u16 ac_current = (((f32)averageTorque / EMRAX_KT) * 1.414 * 10);

    send_ac_current_command(ac_current);
    ifdef TSMS_OVERRIDE{
        send_drive_enable_command(false);
    } else {
        send_drive_enable_command(true);
    }
    dti_set_ac_current(ac_current);
}