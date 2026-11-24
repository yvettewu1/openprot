# Host PIO IBI regression tests

Linux/x86-64 test with a simulated register/FIFO backend. No board, wiring or
firmware image is needed. It compiles the production PIO engine against the model
and checks short/segmented notifications, FIFO draining, overflow, source changes,
errors, polling and IRQ rearming. The IRQ model follows the current
snapshot/acknowledge/rearm APIs; it does not execute the real interrupt handler.

Additional cases repeat six private-write, IBI, private-read sequences in both
polling and subscribed IRQ modes. They compare TX words and RX/IBI bytes, verify
that polling ignores an unrelated stale latch and does not enable signals, and
check target short-RX draining, overflow and live/latched error precedence.
The simulated IRQ path still advances transfers in task context.

Bazel label: `//target/ast10x0/tests/peripherals/i3c_hci/host/pio_ibi:pio_ibi`.
Use the host platform, not `--platforms=//target/ast10x0`. The `manual` tag keeps
this host-only target out of recursive firmware builds. Alternatively, from the
repository root:

```sh
rustc --edition 2024 --test target/ast10x0/tests/peripherals/i3c_hci/host/pio_ibi/host_ibi.rs -o /tmp/hci-ibi-tests
/tmp/hci-ibi-tests
```

These are instructions for later execution; no build/test was run for this refactor.
The mock intentionally targets the PIO engine, not the application traits; those
traits are exercised by the hardware loopback and MCTP tests.
