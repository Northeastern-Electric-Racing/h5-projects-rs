//! The one isoSPI line and chip API this board talks to, named once so the task loop, the cache
//! and the recovery service agree without any of them importing each other.

#[cfg(not(feature = "hil"))]
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{Spi, mode::Master},
};
#[cfg(not(feature = "hil"))]
use embassy_time::Delay;
#[cfg(not(feature = "hil"))]
use embedded_hal_bus::spi::ExclusiveDevice;

/// Must be built with `ExclusiveDevice::new`, not `new_no_delay`: the driver's wake-up pulse is a delay-only transaction and panics without delay support.
#[cfg(feature = "hil")]
pub type SpiDevice = crate::hil::HilDevice;
#[cfg(not(feature = "hil"))]
pub type SpiDevice = ExclusiveDevice<Spi<'static, Async, Master>, Output<'static>, Delay>;

/// The error type our `SpiDevice` produces.
pub type SpiError = <SpiDevice as embedded_hal_async::spi::ErrorType>::Error;

/// Type alias representing one isoSPI line to our ADBMS2950B.
pub type Line = adbms2950::line::Line<SpiDevice>;

/// Type alias for the stateful API over both lines.
pub type Api = adbms2950::api::Api<SpiDevice>;
