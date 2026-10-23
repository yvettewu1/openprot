# AST1080 I3C HCI init smoke test

This bare-metal image runs I3C HCI command/descriptor checks and reads the HCI0
hardware registers. It uses standalone Cortex-M reset/vector code, with no
Pigweed kernel, userspace, semihosting, or tokenized logging.

## Build in WSL

```bash
bazel build --platforms=//target/ast10x0 \
  //target/ast10x0/tests/peripherals/i3c_hci/i3c_init:i3c_hci
```

Outputs: `bazel-bin/target/ast10x0/tests/peripherals/i3c_hci/i3c_init/i3c_hci.elf`
and `i3c_hci.bin` in the same directory.

To build all six images:

```bash
bazel build --platforms=//target/ast10x0 \
  //target/ast10x0/tests/peripherals/i3c_hci/...
```

## Loading and output

The raw binary has no boot header. Load it into SRAM at address zero and start
via its vector table (initial MSP and Thumb reset address). The loader must
configure UART12/BMC-UART clocks, pinmux, baud rate and line format. The shared
console uses `ast1080_pac::Uart5`, which maps to BMC-UART at `0x74C33C00`.
Logs and `TEST_RESULT:PASS` / `TEST_RESULT:FAIL` are plain UART text.

`bare_metal/runtime.ld` defines the memory layout; `bare_metal/runtime.rs`
provides reset, BSS initialization, UART logging, panic handling and IRQ vectors
96/97. Individual IRQ tests register their driver contexts before unmasking the
NVIC. The Cortex-M crate provides single-core critical sections.

The image build labels and `.bin` names are preserved. The old kernel-specific
`system.json5`, generated target glue and Bazel hardware-test wrapper rules are
removed. These targets produce firmware; they do not upload or run it.
