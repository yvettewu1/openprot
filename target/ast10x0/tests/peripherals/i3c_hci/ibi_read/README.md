# External AST2700 IBI-triggered read

Standalone polling test on HCI0 with LV pin setup, primary address 0x08,
and expected target PID 0x07ec06014000. Does not modify the existing local
two-controller `ibi` test or require GETPID CCC reads to pass.

Targets:
- `//target/ast10x0/tests/peripherals/i3c_hci/ibi_read:ibi_read`
- `//target/ast10x0/tests/peripherals/i3c_hci/ibi_read:ibi_read.elf`

1. Stop any AST2700 process holding the selected target node open (the driver
   allows only one client). Keep the existing kernel and device tree.
2. Run this firmware. It performs ENTDAA, accepts payload IBIs in DAT,
   enables SIR using ENEC, then prints `READY` and waits indefinitely.
3. Only after READY, on AST2700 send:

   ```sh
   mctp-i3c -d /dev/i3c-mctp-target-N -w 0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88
   ```

   Use the node mapped to `4-target`; do not assume N equals the bus number.
4. Expect `IBI + PRIVATE READ DATA PASS` followed by `TEST_RESULT:PASS`.

Checks: IBI source, error flag, MDB 0xae, optional MDB PEC (default Linux
workaround), eight-byte private-read completion/length, and exact raw payload.
This test supplies no MCTP header in the read payload. The IBI MDB/PEC is
separate from the read data. The kernel target driver copies userspace TX
bytes; if your mctp-i3c tool adds framing or a data PEC, its actual output must
be verified and EXPECTED_DATA/read length adjusted explicitly. Such app-side
framing has not been validated here and mismatches are not silently accepted.

IBI is polled with CPU interrupts masked. There is no sleep between receiving
the IBI and issuing the read because the Linux pending-read operation times out.
Only the operator wait is unbounded; driver transfer budgets are bounded but
uncalibrated polling iterations. Failed runs dump registers and attempt DISEC
cleanup; disable_ibi is best-effort and does not prove target-side DISEC success.
The firmware builds and the host IBI FIFO regression tests pass. No board
execution or AST2700 kernel modification accompanies this fix.

Discovery matches the working `mctp_write` polling path: primary initialization,
IRQ masking/status setup, register diagnostics, and one `entdaa_all` call.
There is no startup RSTDAA, discovery retry, or added settling delay. Run with
the same target starting state as the working write test; a target retaining
a previously assigned dynamic address may not participate in ENTDAA.
IBI configuration begins only after discovery and target identity selection.

## IBI receive behavior

Before ENEC, the test requests a two-byte IBI receive limit (MDB plus optional
PEC). The ASPEED limit is controller-wide, so an existing larger value is
preserved, matching Linux/Zephyr. The test prints its readback as ASPEED_MST_MRL.
Both ASPEED IBI thresholds are set to one for short notifications. After a
status descriptor is consumed, its payload is drained by length without polling
the status-only threshold bit. Target interrupts are assembled through the final
status, checking source continuity and retaining errors across segments. An
oversized transaction is drained through its final segment and rejected.

The test validates the completed notification before issuing the private read.
It prints packet data only after issuing that read, and reports PASS only after
checking the returned length and all eight bytes. An incomplete or invalid IBI
fails the test rather than triggering a read or reporting success.

Build from the repository root:

```sh
bazel build --config=k_ast1060_evb //target/ast10x0/tests/peripherals/i3c_hci/ibi_read:ibi_read
```

Run the production PIO code against a simulated status-only ASPEED FIFO:

```sh
rustc --edition 2024 --test target/ast10x0/tests/peripherals/i3c_hci/pio/host_ibi.rs -o /tmp/hci-ibi-tests
/tmp/hci-ibi-tests
```

Hardware retesting must confirm a valid MDB, successful private read, and absence
of the target's early-termination/timeouts. Host tests cannot establish bus timing
or diagnose the separate AST2700 RCU stall.

Failed completed transactions log the final raw status, total length, and first
eight payload bytes after draining. Status 0x41001304 has ERROR and LAST set,
source address 0x09, and a four-byte final segment; it must never produce PASS.
