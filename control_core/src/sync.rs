//! Waiting on a future and taking a lock from synchronous code inside the
//! tokio runtime.

use std::future::Future;
use std::sync::{Mutex, MutexGuard, PoisonError};

/// Runs `future` to completion from synchronous code on a runtime worker,
/// such as a service handler that takes a plain closure. `block_in_place`
/// hands this worker's queue to another runtime thread for the duration, so
/// the other tasks keep running while this one waits. Legal only on a
/// multi-thread runtime.
pub fn wait_for<T>(future: impl Future<Output = T>) -> T {
    tokio::task::block_in_place(|| tokio::runtime::Handle::current().block_on(future))
}

/// The guard of `mutex`, also when a holder panicked and poisoned it: the
/// value stays usable as the last writer left it.
pub fn lock_ignoring_poison<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_future_runs_to_completion_inside_a_synchronous_handler() {
        let handler = || wait_for(async { 7 });
        assert_eq!(handler(), 7);
    }

    #[test]
    fn a_poisoned_lock_still_gives_its_value() {
        let shared = Mutex::new(7);
        let poisoner = std::panic::catch_unwind(|| {
            let _guard = shared.lock().unwrap();
            panic!("a writer panics while it holds the lock");
        });
        assert!(poisoner.is_err() && shared.is_poisoned());
        assert_eq!(*lock_ignoring_poison(&shared), 7);
    }
}
