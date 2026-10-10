use core::cell::{Cell, RefCell};
use core::future::poll_fn;
use core::task::Poll;

use embassy_sync::blocking_mutex::raw::{RawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::waitqueue::MultiWakerRegistration;

struct State<T, const N: usize> {
    value: Cell<Option<T>>,
    wakers: RefCell<MultiWakerRegistration<N>>,
}

/// Cell that starts out empty, but then gets filled in early in the program's lifespan. While the
/// cell is still empty, the `.get()` method will .await until the data is filled. Then, once the cell
/// has data, the `.get()` method will return the data immediately.
/// 
/// This is meant to be used as a replacement for `Option` or sentinel/default values for data that needs
/// to be initialized by reading from an outside source (i.e., a sensor or something). If data can be initialized
/// on the MCU directly (or can be const initialized), don't use this. This also probably shouldn't be used for tasks
/// that need to do other unrelated work if this data isn't initialized yet, since .awaiting will sleep the whole
/// task until this data is filled. It should primarily be used for tasks that have no need to progress yet
/// without the data stored in here.
///
/// `N` is the number of waker slots. You should set this to the number of tasks you
/// normally expect to be waiting at the same time. However, if more tasks than `N` try
/// waiting, everything will still work correctly. Registering another task just wakes 
/// all existing registered tasks, who then re-check/re-register. So, `N` is mainly just an efficiency
/// thing you can optimize if you want to. It should never really matter much at all because
/// waiters only exist at init time.
pub struct StartupCellRaw<M: RawMutex, T, const N: usize> {
    inner: Mutex<M, State<T, N>>,
}

impl<M: RawMutex, T: Copy, const N: usize> StartupCellRaw<M, T, N> {
    /// Creates an empty cell.
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(State {
                value: Cell::new(None),
                wakers: RefCell::new(MultiWakerRegistration::new()),
            }),
        }
    }

    /// Stores `value` and replaces any previous value. Upon the first
    /// time being called, this will wake up all waiters.
    pub fn set(&self, value: T) {
        self.inner.lock(|s| {
            if s.value.replace(Some(value)).is_none() {
                s.wakers.borrow_mut().wake();
            }
        });
    }

    /// Gets the current value, or `None` if not set yet.
    pub fn try_get(&self) -> Option<T> {
        self.inner.lock(|s| s.value.get())
    }

    /// Gets the current value. If there is no data inside yet, this
    /// will .await until the data has been set for the first time.
    /// 
    /// Once the data has been set for the first time, this future
    /// will always resolve immediately and basically works like a normal sync function.
    pub async fn get(&self) -> T {
        poll_fn(|cx| {
            self.inner.lock(|s| match s.value.get() {
                Some(v) => Poll::Ready(v),
                None => {
                    s.wakers.borrow_mut().register(cx.waker());
                    Poll::Pending
                }
            })
        })
        .await
    }

    /// Waits for the first write to this cell, and then returns a handle
    /// that provides a nicer API for getting data that is known to
    /// already be set (i.e., no need to .await).
    pub async fn ready(&self) -> Ready<'_, M, T, N> {
        self.get().await;
        Ready { cell: self }
    }
}

/// Handle for data that is known to be set already.
pub struct Ready<'cell, M: RawMutex, T, const N: usize> {
    cell: &'cell StartupCellRaw<M, T, N>,
}

impl<M: RawMutex, T, const N: usize> Clone for Ready<'_, M, T, N> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<M: RawMutex, T, const N: usize> Copy for Ready<'_, M, T, N> {}

impl<M: RawMutex, T: Copy, const N: usize> Ready<'_, M, T, N> {
    /// Gets the current value.
    pub fn get(&self) -> T {
        match self.cell.try_get() {
            Some(v) => v,

            // PANIC SAFETY: The API ensures that a `Ready` is only created after
            // `Some` is observed, and also ensures that nobody can write `None` back
            // into the cell.
            None => unreachable!("StartupCellRaw became empty after being set"),
        }
    }
}

/// Simplified StartupCell type for use in this project.
pub type StartupCell<T> = StartupCellRaw<ThreadModeRawMutex, T, 10>;