# AST1040 DMA loopback

Connect HCI0 SCL to HCI1 SCL and HCI0 SDA to HCI1 SDA at compatible I/O voltage levels. Isolate these lines from AST2700 or any other controller. This is an external wire loopback on one AST1040, not an internal loopback mode.

HCI0 uses DMA for discovery, ENEC, private writes, private reads and IBI reception. HCI1 uses the existing PIO target driver. This tests controller DMA; it does not test target DMA. The selected ports are HCI0 and HCI1 with the existing LV board setup and explicit pinmux groups.

Firmware target: `//target/ast10x0/tests/peripherals/i3c_hci/dma_loopback:dma_loopback` (ELF target suffix `.elf`). Linker layout reserves the same DMA SRAM region as dma_test.

The test discovers the local target by PID, checks its assigned address, enables IBI, then runs six rounds of eight-byte private write/read and two-byte IBI (`ae 2b`). Each round verifies payloads and consumes target completions. Six rounds exercise command-ring wraparound. Waits are bounded polling iterations, not measured microseconds. CPU interrupts remain masked; HCI global/ring signaling is enabled.

Success requires all phases and final `TEST_RESULT:PASS`. Unlike the separate dma_test diagnostic, this test does not ignore ENEC failure. The existing dma_test response bypass is unchanged. A missing second DMA response may therefore stop this test at ENEC; that is a failed loopback result, not a PASS.

Source has not been built or run on hardware.

Target service drains ENTDAA/ENEC notifications before private transfers and checks the target hardware SIR-enable state. For IBI, MDB `ae` is embedded in target command `0x0001ae49`; only byte `2b` is queued in the TX FIFO. The IBI request is started without blocking, then both the controller DMA event and target completion are polled with a bounded wait. Target FIFO status bits stay enabled with one-DWORD thresholds for short CCCs. DAA and standard CCC state changes are performed by target hardware; the test validates notifications rather than emulating those commands in software.
