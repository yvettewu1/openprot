// Licensed under the Apache-2.0 license
// SPDX-License-Identifier: Apache-2.0
//! Scheduling policy shared by synchronous adapters. No MMIO or FIFO ownership.
//! Budgets count unsuccessful probes, not elapsed microseconds. Kernel waits can
//! drive the same progress functions later without consuming descriptors in ISRs.
/// An in-flight operation owns its engine and buffers until the adapter returns.
/// Kernel wakeups can drive advance without giving FIFO/ring ownership to ISRs.
/// A task wait must arm/register its wakeup before rechecking advance, then
/// block only while still pending. Some DMA phases (idle/writeback) need a
/// bounded recheck without a new IRQ. Complete is terminal; do not advance again.
pub(crate) trait Operation {
    type Output;
    type Error;
    fn advance(&mut self) -> Result<Progress<Self::Output>, Self::Error>;
    /// Bare-metal polling only. IRQ/task adapters choose their own wait mechanism.
    fn on_poll_pending(&mut self) {}
}
pub(crate) struct Polling {
    pub budget: u32,
}
impl Polling {
    pub fn run<O: Operation>(
        &self,
        mut operation: O,
        timeout: O::Error,
    ) -> Result<O::Output, O::Error> {
        drive(
            self.budget,
            || {
                let progress = operation.advance()?;
                if matches!(&progress, Progress::Pending) {
                    operation.on_poll_pending();
                }
                Ok(progress)
            },
            timeout,
        )
    }
}

pub(crate) enum Progress<T> {
    Pending,
    Advanced,
    Complete(T),
}

pub(crate) fn drive<T, E>(
    attempts: u32,
    mut step: impl FnMut() -> Result<Progress<T>, E>,
    timeout: E,
) -> Result<T, E> {
    let budget = attempts.max(1);
    let mut remaining = budget;
    loop {
        match step()? {
            Progress::Complete(value) => return Ok(value),
            Progress::Advanced => remaining = budget,
            Progress::Pending => {
                remaining -= 1;
                if remaining == 0 {
                    return Err(timeout);
                }
                core::hint::spin_loop();
            }
        }
    }
}

pub(crate) fn poll<T, E>(
    attempts: u32,
    mut step: impl FnMut() -> Result<Option<T>, E>,
    timeout: E,
) -> Result<T, E> {
    drive(
        attempts,
        || {
            step().map(|result| match result {
                Some(value) => Progress::Complete(value),
                None => Progress::Pending,
            })
        },
        timeout,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn progress_resets_only_the_wait_budget() {
        let mut calls = 0;
        let result = drive(
            2,
            || {
                calls += 1;
                Ok::<_, ()>(match calls {
                    1 | 3 => Progress::Pending,
                    2 => Progress::Advanced,
                    _ => Progress::Complete(7),
                })
            },
            (),
        );
        assert_eq!(result, Ok(7));
        assert_eq!(calls, 4);
    }
    #[test]
    fn pending_is_bounded_and_step_errors_propagate() {
        let mut calls = 0;
        assert_eq!(
            poll::<(), _>(
                3,
                || {
                    calls += 1;
                    Ok(None)
                },
                "timeout"
            ),
            Err("timeout")
        );
        assert_eq!(calls, 3);
        assert_eq!(
            poll::<(), _>(3, || Err("hardware"), "timeout"),
            Err("hardware")
        );
    }
}

#[cfg(test)]
mod operation_tests {
    use super::*;
    use core::cell::Cell;
    struct Pending<'a>(&'a Cell<bool>);
    impl Operation for Pending<'_> {
        type Output = ();
        type Error = ();
        fn advance(&mut self) -> Result<Progress<()>, ()> {
            Ok(Progress::Pending)
        }
    }
    impl Drop for Pending<'_> {
        fn drop(&mut self) {
            self.0.set(true);
        }
    }
    #[test]
    fn polling_timeout_releases_the_operation_guard() {
        let cancelled = Cell::new(false);
        assert_eq!(Polling { budget: 1 }.run(Pending(&cancelled), ()), Err(()));
        assert!(cancelled.get());
    }
}

#[cfg(test)]
mod scheduling_tests {
    use super::*;
    struct Steps {
        probes: usize,
        yields: usize,
    }
    impl Operation for Steps {
        type Output = usize;
        type Error = ();
        fn advance(&mut self) -> Result<Progress<usize>, ()> {
            self.probes += 1;
            Ok(match self.probes {
                1 => Progress::Pending,
                2 => Progress::Advanced,
                _ => Progress::Complete(self.yields),
            })
        }
        fn on_poll_pending(&mut self) {
            self.yields += 1;
        }
    }
    #[test]
    fn polling_yields_only_on_pending_and_progress_can_run_without_yield() {
        assert_eq!(
            Polling { budget: 2 }.run(
                Steps {
                    probes: 0,
                    yields: 0
                },
                ()
            ),
            Ok(1)
        );
        let mut task = Steps {
            probes: 0,
            yields: 0,
        };
        assert!(matches!(task.advance(), Ok(Progress::Pending)));
        assert!(matches!(task.advance(), Ok(Progress::Advanced)));
        assert!(matches!(task.advance(), Ok(Progress::Complete(0))));
    }
}
