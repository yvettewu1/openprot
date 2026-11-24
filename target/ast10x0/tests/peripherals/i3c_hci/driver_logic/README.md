# Driver logic checks

`//target/ast10x0/tests/peripherals/i3c_hci/driver_logic:driver_logic`
produces `driver_logic.elf` and `driver_logic.bin`. It replaces `pio:pio` and
consolidates the pure checks formerly repeated in `i3c_init`.

This is still a bare-metal firmware image: it needs AST1040 and UART to run,
but no external I3C target or loopback wires. The test body performs no HCI
register access, resets, pin setup or bus traffic. It checks:

- HCI version/capability and synthetic section decoding.
- V1/V2 command encoding, rates, CCC definitions and target CCC classification.
- Byte packing, DAT/DCT parsing, address/slot allocation and device-table updates.
- DAA command planning and response classification.
- Transfer-buffer validation and response decoding.

Low-level pure APIs are intentional here: testing encoders through the controller
trait would require hardware and would stop checking the exact descriptor words.
Real PIO transfers are tested by `../loopback/pio`; FIFO IBI regressions use
`../host/pio_ibi`. These logic checks do not prove hardware V2 support.

Uses the shared bare-metal runtime/linker and emits `TEST_RESULT:PASS` or `FAIL`.
