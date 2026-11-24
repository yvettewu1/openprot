# AST1040 PIO/PIO loopback

Uses the dedicated `CONTROLLER_BUS` and `TARGET_BUS` from the
[board configuration guide](../../README.md), defaulting to HCI0 and HCI1.
The test selects PIO for the controller and PIO for the target before
initialization; no `config.rs` edit is needed when switching tests. Roles,
identity, timing and DMA reservations remain board-configured. Connect the SCL
pins of those ports and their SDA pins at compatible voltage levels, isolated
from AST2700. The selected hardware bus numbers are printed at startup.

Firmware target: `//target/ast10x0/tests/peripherals/i3c_hci/loopback/pio:pio_pio`
(ELF target suffix `.elf`).

The test uses the configured target PID for discovery, validates its assigned
address and ENEC, then runs six rounds of eight-byte private write/read and
two-byte IBI (`ae 2b`). It checks both payloads and target completions. Six rounds
exercise DMA command-ring wraparound where DMA is selected. Target hardware
executes DAA and standard CCCs; software drains their notifications.

Waits are bounded polling iterations, not measured microseconds. CPU interrupts
remain masked. All phases must succeed before `TEST_RESULT:PASS`; this loopback
does not use the separate MCTP DMA diagnostic's ENEC response bypass.

Protocol checks now use `I3cController` / `I3cTarget`. The shared lifecycle owns
board setup, discovery state and cleanup; tests do not manage DAT entries or
DMA rings. The selected transport, payloads and transfer ordering are retained.
