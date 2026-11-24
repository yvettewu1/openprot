# Command V2 DMA loopback

Uses the dedicated `CONTROLLER_BUS` and `TARGET_BUS` from the
[board configuration guide](../../README.md), defaulting to HCI0 and HCI1.
Both transports are selected as DMA before initialization, independently of the
board defaults. Connect SCL and SDA between the selected ports printed at
startup, isolated from other controllers. Switching tests requires no config edit.

The test checks the configured controller command-format capability before starting either DMA ring. AST1040 hardware advertising V1 reports `TEST_RESULT:SKIP`, not PASS. Never force V2 descriptors on V1 hardware.

Discovery publishes the V2 PID-read/address-assignment pair together through DMA, including terminal-NACK cleanup. The same DMA ring then handles ENEC, six rounds of private write, target IBI (`ae 2b`), and private read. Six rounds exercise ring wraparound. Target commands retain their separate one-word target encoding; this test exercises controller command V2.

Controller DMA requires at least 28 bytes per entry: four command words plus three buffer-descriptor words. Hardware-provided larger strides are retained. SRAM DMA mapping, cache configuration and target-side support remain board-specific.

V2 discovery and ordinary commands do not fall back to PIO. The ENEC after discovery exercises ring reuse after the terminal DAA NACK. No build or hardware validation was performed when adding this test.

Protocol checks now use `I3cController` / `I3cTarget`. The shared lifecycle owns
board setup, discovery state and cleanup; tests do not manage DAT entries or
DMA rings. The selected transport, payloads and transfer ordering are retained.
V1 and V2 use the same `bare_metal::i3c_loopback::dma_rounds` assertions.
