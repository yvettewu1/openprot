# AST1040 DMA controller echo

AST1040 uses `config::CONTROLLER_BUS` (HCI0 by default), DMA, and command
format V1 to receive arbitrary target data and write the same bytes back.
AST2700 must be an I3C target. The exchange is shared with
[`controller_pio`](../controller_pio/README.md) in
`bare_metal/i3c_mctp_controller.rs`.

Firmware labels:

- `//target/ast10x0/tests/peripherals/i3c_hci/mctp/controller_dma:controller_dma`
- `//target/ast10x0/tests/peripherals/i3c_hci/mctp/controller_dma:controller_dma.elf`

The test discovers the MCTP target, enables IBI, then runs six
`IBI -> DMA private read -> DMA private write` exchanges. It prints the
actual RX and TX bytes. The old eight-byte pattern check is removed: the write
uses exactly the read's returned length, without adding a header, padding, or
data PEC. Use nonempty packets up to the 128-byte test buffer, subject to the
peer/tool/hardware limits.

After `READY`, run on the matching AST2700 target node:

```sh
mctp-i3c -d /dev/i3c-mctp-target-N \
  -w 0xaa,0xee,0x33,0x44,0x55,0x66,0x77,0x88 \
  -r -c 6 -m 10000
```

Keep `-w` before `-r` in one foreground invocation. Change the write bytes
as needed; use six sequential `-c 1` invocations for six different packets.
The echo should have the same length and bytes as the packet sent. The test
stops after six total exchanges and must be reloaded for another run.

## Retained DMA diagnostics

The ENEC response bypass remains: enable errors are logged and the test
continues waiting for IBI. Normal driver response validation and command-fault
protection remain active, so the bypass cannot guarantee subsequent transfers.
The test still requires a two-byte notification beginning with MDB `ae`,
ignores other source addresses, and does not separately check the second IBI
byte as PEC. PIO retains its separate optional-PEC validation policy.

No initial application write or setup delay is used. The private read is issued
before any payload logging because the Linux target's pending-read operation
has a timeout. IBI waiting is indefinite; transfers use bounded polling budgets.
CPU interrupts remain masked and DMA events are polled.

The existing DMA stop-only cleanup is retained: this test adds no DISEC command.
The shared runner always attempts ring shutdown on returned success or failure,
then disables signaling. A failed DMA stop retains its memory reservation.

Expect `ROUND 0 DMA ECHO PASS` through `ROUND 5 DMA ECHO PASS` followed by
`TEST_RESULT:PASS`. PASS checks transfers, not payload content or an MCTP
endpoint stack. No build, test execution, or hardware validation was performed
for this update.
