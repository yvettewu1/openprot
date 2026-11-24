# AST1040 PIO controller echo

AST1040 is the controller on `config::CONTROLLER_BUS` (HCI0 by default).
AST2700 is an I3C **target**. This test waits for target data and writes the
same bytes back through the generic `I3cController` API. The common protocol
is in `bare_metal/i3c_mctp_controller.rs`, shared with `controller_dma`.

Firmware labels:

- `//target/ast10x0/tests/peripherals/i3c_hci/mctp/controller_pio:controller_pio`
- `//target/ast10x0/tests/peripherals/i3c_hci/mctp/controller_pio:controller_pio.elf`
- Optional ISR-latch mode: `--//target/ast10x0/tests/peripherals/i3c_hci/mctp/controller_pio:irq=true`

## Exchange

1. Discover once, select a target with DCR `cc` and BCR IBI/payload bits, and
   enable SIR with a two-byte notification limit.
2. Print `READY` and wait indefinitely for AST2700 to send data.
3. Validate the IBI source, MDB `ae`, and optional IBI PEC.
4. Immediately perform a private read before printing its notification/data.
5. Print `MCTP RX len=... data=[...]`, then private-write exactly those bytes
   back to the same target. Print `MCTP TX len=... data=[...]` on completion.
6. Repeat for six packets, attempt IBI disable, and shut down the controller.

There is no initial fixed write or receiver-setup delay. Both the read and echo
use the actual received length. Packets may contain any byte values, including
zero bytes; no content comparison, MCTP header insertion, data PEC generation,
padding, or fixed reply pattern is applied. The test buffer is 128 bytes.
Use nonempty packets within that capacity and the peer/tool/hardware limits.

## AST2700 command

Connect the dedicated AST1040 controller port to the AST2700 target port.
Use the matching `/dev/i3c-mctp-target-N` node after `READY`:

```sh
mctp-i3c -d /dev/i3c-mctp-target-N \
  -w 0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88 \
  -r -c 6 -m 10000
```

Use `-w` **before** `-r` in the same foreground invocation. AST2700 queues
the data and raises IBI; AST1040 reads it, then writes it back. The target
character driver queues the echo for the same open client. Opening separate
write/read clients can lose that packet, and concurrent opens are rejected.
The [ASPEED target driver](https://github.com/AspeedTech-BMC/linux/blob/aspeed-master-v6.18/drivers/i3c/mctp/i3c-target-mctp.c)
copies received data into the client's RX queue without changing its bytes.

Change the example data freely. For six different packets, run six commands
sequentially with `-c 1` and different `-w` lists. The tool may display
the first four bytes as a header; the firmware echoes the whole received packet.
After six total echoes, reload the firmware with the target ready for discovery
to repeat. There is no startup RSTDAA or ENTDAA retry.

## Completion and cleanup

Expect `ROUND 0 PIO ECHO PASS` through `ROUND 5 PIO ECHO PASS` and
`TEST_RESULT:PASS`. PASS establishes controller transfer completion, not
target-application content verification. This is a raw transport diagnostic,
not an EID/PLDM endpoint implementation.

The driver rejects clipped reads; the test rejects empty reads and lengths
outside its buffer before writing. No console dump or sleep precedes the read
after a valid IBI. Individual transfers use bounded polling budgets; waiting
for each new operator packet is indefinite.

IBI disable is attempted after ENEC, read/write, or notification failure and
after success. The first protocol error takes precedence; cleanup errors are
logged. DISEC is best-effort in the driver. The shared controller runner then
stops the session and disables signaling on all returned paths.

NVIC is masked by default. The optional IRQ setting retains the existing
ISR-latch/task-polling path and shared registration/cleanup; it is not a Pigweed
sleeping/wakeup test. No build, test execution, or hardware validation was
performed for the echo update.
