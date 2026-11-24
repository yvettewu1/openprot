# AST1040 PIO target / AST2700 MCTP controller

This is the PIO counterpart of [target_dma](../target_dma/README.md).
AST1040 uses the dedicated `config::TARGET_BUS` (HCI1 by default) with PIO
selected before initialization. No board-config edit is required when switching
between the two images. AST2700 remains the I3C controller.

Firmware labels:

- `//target/ast10x0/tests/peripherals/i3c_hci/mctp/target_pio:target_pio`
- `//target/ast10x0/tests/peripherals/i3c_hci/mctp/target_pio:target_pio.elf`

Both target tests call `bare_metal::i3c_mctp_target` for the same six-round
exchange, board setup, completion checks, and cleanup. The PIO entry point
selects `IoMode::Pio`; it does not initialize a DMA ring or start a local
controller. Identity, clocks, timing, and pins come from the board configuration.
The driver handles PIO FIFO/response access behind the generic `I3cTarget` API.

## Hardware and command

Connect AST1040's configured target SCL/SDA to the AST2700 controller port using
compatible voltage and a common ground. Remove local HCI0/HCI1 loopback jumpers.
Use the same wiring, master-mode configuration, discovery, and device selection
as `target_dma`. Load the firmware, wait for `READY`, then repeat AST2700
discovery so it assigns an address and enables SIR.

Use the AST2700 master character node `/dev/i3c-mctp-N` matching the printed
AST1040 PID, not a `/dev/i3c-mctp-target-N` node:

```sh
mctp-i3c -d /dev/i3c-mctp-N \
  -w 0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88 \
  -r -c 6 -m 10000
```

Keep write and read in the same foreground invocation. To send different data,
use six sequential invocations with `-c 1` and a different `-w` list each
time. Wait for each command to return before starting the next; only one
process may hold the device open.

## What it exercises

For each request, AST1040:

1. Receives a private write through the PIO RX FIFO and prints
   `MCTP RX len=... data=[...]`. No fixed packet length or byte-pattern
   comparison is made. Use nonempty packets within the 128-byte application
   buffer and the hardware/negotiated limits in both directions.
2. Calculates `ae <PEC>` from its assigned I3C address.
3. Queues the IBI command and the received bytes as the reply, then raises IBI.
   Both commands are queued before AST2700 can respond to that request.
4. Waits for successful IBI completion and read completion with zero remaining
   bytes. The Linux controller's IBI handler performs the SDR private read;
   userspace `-r` retrieves the packet collected by that handler.

The reply has exactly the received bytes and length, without an added header,
padding, or data PEC. Hardware-serviced CCC notifications are drained while
waiting for requests. A next request arriving before the previous TX completions
are consumed is copied into a separate pending buffer, so later CCC notifications
cannot overwrite its bytes. Unexpected, duplicate, failed, or incomplete completions fail the test.

Expect `MCTP PIO TARGET` at startup, `ROUND 0 PASS` through
`ROUND 5 PASS` with PIO RX/TX labels, then `TEST_RESULT:PASS`.
PASS checks transfers and completion events, not payload correctness or MCTP
endpoint behavior. The test is a raw transport diagnostic, not an EID/PLDM stack.

Operator/request waits are indefinite. After raising IBI, completion is bounded
by `COMPLETION_POLLS` polling iterations, not a wall-clock timer.
NVIC remains masked: this tests bus IBI with polling, not a CPU interrupt.
Every returned exchange result attempts session shutdown; peripheral signaling
is disabled before returning. After six rounds or a failure, reload the test
and repeat controller discovery.

No build, test execution, or hardware validation was performed when adding this
test. In particular, the external-controller PIO sequence with IBI and read
commands queued together still requires board validation.
