# MCTP private-write test

Moved from `master_multi` without changing the working transfer sequence.
Enumerates targets and sends the eight application bytes
`11 22 33 44 55 66 77 88`, wrapped in the existing MCTP vendor-defined message
and address-dependent PEC, using a private write. This is not an unframed raw
transfer. No private reads or IBI are used.

Target labels:
- `//target/ast10x0/tests/peripherals/i3c_hci/mctp_write:mctp_write`
- `//target/ast10x0/tests/peripherals/i3c_hci/mctp_write:mctp_write.elf`
- IRQ option: `--//target/ast10x0/tests/peripherals/i3c_hci/mctp_write:irq=true`

On AST2700, start the receiver on the node mapped to `4-target` before running:
`mctp-i3c -d /dev/i3c-mctp-target-N -r -m 0`.
The existing long, uncalibrated spin delay is retained to allow receiver setup.
`MCTP WRITE BUS PASS` establishes controller completion only; compare the
received bytes on AST2700 to establish end-to-end delivery.
At DA 0x09 the transmitted bytes are
`01 00 08 c8 7e 1a 03 11 22 33 44 55 66 77 88 82`.
