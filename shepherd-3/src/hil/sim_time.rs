//! Reads the Simulink model's own clock over UART and hands it to [`crate::helpers::time`], which owns it.
//! On a rig that clock is the plant clock, so everything measured in plant time runs off this one read.

use super::{Async, READ_TIMEOUT, UartRx, UartTx, with_timeout};
use adbms2950::chip::{
    pec::{CommandPec, DataPecRx},
    registers::{ReadableGroup, results::AccumulatedCurrentAndBatteryVoltage},
};

const RDHILTIME: u16 = 0b111_1111_1111;

/// The wire frame for [`RDHILTIME`]: opcode then command PEC, same as any other command.
const fn query() -> [u8; 4] {
    let command = [(RDHILTIME >> 8_i32) as u8, RDHILTIME as u8];
    let pec = CommandPec::new(&command);

    [command[0], command[1], pec.pec0(), pec.pec1()]
}

/// Reads the rig's clock, but only after the one read whose timestamp is load bearing -- gating it elsewhere would cost a frame per register group for nothing.
/// The reply lags the SNAP by a couple of frames, which cancels because the accumulator only ever differences two stamps.
pub(super) async fn refresh_after(command: &[u8], tx: &mut UartTx<'static, Async>, rx: &mut UartRx<'static, Async>) {
    if command != AccumulatedCurrentAndBatteryVoltage::READ_COMMAND.to_bytes().as_slice() {
        return;
    }

    let mut reply = [0_u8; 8];
    let fetched = with_timeout(READ_TIMEOUT, async {
        tx.write(&query()).await?;
        tx.flush().await?;
        rx.read(&mut reply).await
    })
    .await;

    if !matches!(fetched, Ok(Ok(()))) {
        defmt::warn!("HvPlate: hil: the rig did not answer RDHILTIME; everything measured in plant time is holding against a stale simulation clock.");
        return;
    }

    if !DataPecRx::from_bytes([reply[6], reply[7]]).verify(&reply[..6]) {
        defmt::warn!("HvPlate: hil: RDHILTIME failed its PEC; everything measured in plant time is holding against a stale simulation clock.");
        return;
    }

    // Six bytes of microseconds, little endian. 48 bits is about nine years of simulated time.
    let mut micros: u64 = 0;
    for (index, byte) in reply[..6].iter().enumerate() {
        micros |= u64::from(*byte) << (index * 8);
    }

    if let Some(previous) = crate::helpers::time::report_plant_micros(micros)
        && micros < previous
    {
        defmt::error!("HvPlate: hil: the rig's clock went backwards ({=u64} us -> {=u64} us). Every deadline and dwell measured in plant time is now stale; restart the firmware along with the model.", previous, micros);
    }
}
