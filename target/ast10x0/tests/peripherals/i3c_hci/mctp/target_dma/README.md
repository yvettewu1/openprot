# AST1040 DMA target / AST2700 MCTP master

This standalone test uses `config::TARGET_BUS` (HCI1 by default), its configured
identity/timing/memory, and DMA for incoming writes, outgoing IBI and read data.
AST2700 is the external controller. It uses the public `I3cTarget` trait for the
exchange and the shared bare-metal linker script. HCI0 is not started as a local
controller; remove the HCI0/HCI1 loopback jumpers.

The exchange and lifecycle now live in
`bare_metal/i3c_mctp_target.rs`, shared with
[`target_pio`](../target_pio/README.md). This entry point explicitly selects
`IoMode::Dma`. Both target tests echo the received bytes and actual length;
the six rounds, IBI/read queue order, polling budgets, and shutdown sequence
are retained.

This is a **raw character-device transport diagnostic**. It prints incoming
MCTP data without checking it against an expected length or byte pattern. The
test receive buffer holds up to 128 bytes; use nonempty packets within that
capacity and the negotiated/hardware limits in both directions. The reply has
exactly the received length and bytes, with no added header, padding, or data PEC.
It does not implement MCTP endpoint discovery, EID assignment, PLDM,
fragmentation, or packet data PEC. The two-byte MCTP IBI is
`ae <PEC>`; its PEC is calculated from the assigned I3C address, not hardcoded
(`ae 2b` at DA `09`). Use the ASPEED master character-device path with raw data
PEC handling, not a network MCTP endpoint or a PEC-enforcing packet stack.

## Hardware and startup

1. Connect the selected AST1040 target SCL/SDA to the AST2700 controller port,
   with compatible voltage and a common ground. Configure that AST2700 port as
   **master**, not target. Keep other controllers off these wires.
2. Load the test and wait for `READY`. Then run discovery on AST2700 using your
   board's controller initialization/DAA flow. The firmware waits indefinitely
   for setup and the first request; it does not issue Hot Join automatically.
   If AST2700 discovered the bus before the firmware started, repeat discovery.
3. Confirm the discovered PID matches the firmware's printed board-configured
   PID and binds to the ASPEED `i3c-mctp` master driver. The board identity must
   have DCR `cc` and BCR IBI/payload bits set (the default target already does).
   The master must enable IBI/SIR and configure MRL/MWL. The target drains CCC
   notifications while waiting; hardware handles those CCCs. Linux discovery
   also generates CCC-read notifications (for example GETMWL/GETMRL). These
   consume a DMA status entry but no RX data chunks; the generic API reports
   `CccReceived` with `len=0`. They are not private-read completions.
4. Select the corresponding `/dev/i3c-mctp-N`, **not**
   `/dev/i3c-mctp-target-N`. Stop any other client using that character device.

Target label (for your own build/flash workflow):

```text
//target/ast10x0/tests/peripherals/i3c_hci/mctp/target_dma:target_dma
```

## Run on AST2700

Replace `N` with the device belonging to the AST1040 target. The write bytes
below are only an example; send different data or lengths as needed:

```sh
mctp-i3c -d /dev/i3c-mctp-N \
  -w 0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88 \
  -r -c 6 -m 10000
```

Keep write and read in **one invocation**, in that order. The ASPEED tool
executes these operations on one open descriptor and repeats them six times.
Opening the master character device again flushes its receive queue, so separate
write/read invocations could discard an already received response. See the
[ASPEED tool](https://github.com/AspeedTech-BMC/aspeed_app/blob/master/mctp-i3c/mctp-i3c.c)
and [master character driver](https://github.com/AspeedTech-BMC/linux/blob/aspeed-master-v6.18/drivers/i3c/mctp/i3c-mctp.c).
Do not use `-t`/`-v`: those options generate a different test protocol.

The target prints `MCTP RX len=... data=[...]` and
`MCTP TX len=... data=[...]` with the same bytes and length. For the example
above, the reply is `11 22 33 44 55 66 77 88`. There is no fixed reply.
The tool may display the first four bytes as a header and the rest as payload;
the whole received packet is echoed. No payload pattern is validated.

To send six different packets, use six sequential foreground invocations with
`-c 1` and different `-w` lists, keeping `-w ... -r` together each time.

## Firmware checks and completion

For each of six rounds, the target displays the incoming length and bytes,
queues IBI then the received packet's read-data descriptor, and only then raises IBI. This follows the
[ASPEED target pending-read ordering](https://github.com/AspeedTech-BMC/linux/blob/aspeed-master-v6.18/drivers/i3c/master/mipi-i3c-hci/core.c).
Both descriptors are ready before the master reacts to the IBI. The Linux master
IBI callback performs the private read; userspace `-r` consumes its queued packet.

The target requires successful IBI completion and read completion with zero
remaining bytes. It also retains a next-round request that arrives before the
previous TX completion is consumed. A separate pending-packet buffer preserves
those bytes while later CCC events reuse the RX scratch buffer. The driver
copies queued TX data before that pending buffer can be reused. Six rounds
exercise descriptor-ring wrap.
It prints `ROUND 0` through `ROUND 5`, stops target DMA, and then prints
`TEST_RESULT:PASS`. PASS confirms target-side RX and successful IBI/TX completion,
not payload correctness. Inspect the displayed data on either board as needed.

Request/operator waits are indefinite. Once an IBI is raised, completion uses
`COMPLETION_POLLS` iterations, not a wall-clock timeout. CPU interrupts remain
masked; this tests IBI on the I3C bus with DMA completion polling, not an ISR.
On returned failures the session attempts DMA shutdown and masks global signals.
A failed stop keeps the driver's memory reservation and is logged. Reset/reload
the firmware and repeat master discovery for another six-round run.
