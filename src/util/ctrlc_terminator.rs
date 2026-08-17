use crate::util::terminator::Terminator;
use jagua_rs::Instant;
use log::warn;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Once;
use std::time::Duration;

/// Number of times Ctrl-C has been pressed (process-wide, incremented by the signal handler)
static CTRLC_COUNT: AtomicUsize = AtomicUsize::new(0);
/// Guards the (one-time) registration of the Ctrl-C handler
static CTRLC_HANDLER: Once = Once::new();

/// Terminator that triggers on a timeout or when Ctrl-C is pressed.
///
/// Multiple instances can coexist (e.g. one per parallel run): the signal handler is registered only once,
/// and every instance compares the global Ctrl-C counter against the value it observed at its last `new_timeout`.
/// Pressing Ctrl-C therefore terminates the *current* phase of every active terminator, without instances
/// interfering with each other.
#[derive(Debug, Clone)]
pub struct CtrlCTerminator {
    pub timeout: Option<Instant>,
    /// Value of the global Ctrl-C counter when the timeout was last (re)set
    ctrlc_count_at_reset: usize,
}

impl Default for CtrlCTerminator {
    fn default() -> Self {
        Self::new()
    }
}

impl CtrlCTerminator {
    /// Creates a new terminator, setting up the process-wide Ctrl-C handler on first use.
    pub fn new() -> Self {
        CTRLC_HANDLER.call_once(|| {
            ctrlc::set_handler(move || {
                warn!(" terminating...");
                CTRLC_COUNT.fetch_add(1, Ordering::SeqCst);
            }).expect("Error setting Ctrl-C handler");
        });

        Self {
            timeout: None,
            ctrlc_count_at_reset: CTRLC_COUNT.load(Ordering::SeqCst),
        }
    }
}

impl Terminator for CtrlCTerminator {
    fn kill(&self) -> bool {
        self.timeout.is_some_and(|timeout| Instant::now() > timeout)
            || CTRLC_COUNT.load(Ordering::SeqCst) > self.ctrlc_count_at_reset
    }

    fn new_timeout(&mut self, timeout: Duration){
        // Ignore any Ctrl-C presses so far and set a new timeout
        self.ctrlc_count_at_reset = CTRLC_COUNT.load(Ordering::SeqCst);
        self.timeout = Some(Instant::now() + timeout);
    }

    fn timeout_at(&self) -> Option<Instant> {
        self.timeout
    }
}
