# Register smoke test

`//target/ast10x0/tests/peripherals/i3c_hci/register_smoke:register_smoke`
produces `register_smoke.elf` and `register_smoke.bin`. It replaces `i3c_init:i3c_hci`.

Run on AST1040 without an external target. The firmware selects the dedicated
`config::CONTROLLER_BUS`, prepares the board, and validates actual HCI register
and section decoding. It does not enable discovery or issue bus transfers.
Hardware preparation resets all HCI instances, so use it as a standalone image.

Pure encoding, configuration parsing and validation checks now live in
`../driver_logic`; they are not repeated here. Register access intentionally
uses the PAC-backed register API because registers are the subject of this test.

The shared `bare_metal/runtime.ld` and runtime provide SRAM startup and plain
UART PASS/FAIL reporting. Load at address zero using the vector table, with the
board loader's UART configuration in place. No kernel is needed.
