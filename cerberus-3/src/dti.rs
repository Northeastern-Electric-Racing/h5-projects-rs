extern crate uom;

const SAMPLES       = 3
const TIRE_DIAMETER = 16 /* INCHES */
const GEAR_RATIO    = (39.0f / 13.0f)
const POLE_PAIRS    = 10
const EMRAX_KT      = 0.61

struct MovingAverage {
    torqueBuffer: f32[SAMPLES],
    torqueIndex: u8,
    regenBuffer: u16[SAMPLES],
    regenIndex: u8
} 

