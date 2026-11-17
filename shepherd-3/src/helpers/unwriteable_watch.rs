//! A read-only view of an embassy [`Watch`].

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::watch::{AnonReceiver, Receiver, Watch};

/// Watch that lets you observe values, but not publish them.
#[repr(transparent)]
pub struct UnwriteableWatch<M: RawMutex, T: Clone, const N: usize> {
    inner: Watch<M, T, N>,
}

impl<M: RawMutex, T: Clone, const N: usize> UnwriteableWatch<M, T, N> {
    /// Narrows a [`Watch`] to its reading half, withholding `sender`.
    pub const fn of(watch: &Watch<M, T, N>) -> &Self {
        // SAFETY: `#[repr(transparent)]` over `Watch`, so identical layout. Same pattern as `StartupCell::to_unwriteable`.
        unsafe { &*(watch as *const Watch<M, T, N> as *const Self) }
    }

    /// The latest value, or `None` if nothing has been published. Takes no receiver slot.
    pub fn try_get(&self) -> Option<T> {
        self.inner.try_get()
    }

    /// A receiver that can `await` each new value. `None` once `N` of them exist.
    pub fn receiver(&self) -> Option<Receiver<'_, M, T, N>> {
        self.inner.receiver()
    }

    /// An observer that cannot `await`, only poll. Unlimited, and takes no slot.
    pub fn anon_receiver(&self) -> AnonReceiver<'_, M, T, N> {
        self.inner.anon_receiver()
    }
}
