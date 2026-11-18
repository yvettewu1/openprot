# Command V2 DMA loopback

HCI0 is the controller; HCI1 is the DMA target. Connect SCL0 to SCL1 and SDA0 to SDA1, isolated from other controllers.

The test checks HCI0 command-format capability before starting either DMA ring. AST1040 hardware advertising V1 reports `TEST_RESULT:SKIP`, not PASS. Never force V2 descriptors on V1 hardware.

Discovery publishes the V2 PID-read/address-assignment pair together through DMA, including terminal-NACK cleanup. The same DMA ring then handles ENEC, six rounds of private write, target IBI (`ae 2b`), and private read. Six rounds exercise ring wraparound. Target commands retain their separate one-word target encoding; this test exercises controller command V2.

Controller DMA requires at least 28 bytes per entry: four command words plus three buffer-descriptor words. Hardware-provided larger strides are retained. SRAM DMA mapping, cache configuration and target-side support remain board-specific.

V2 discovery and ordinary commands do not fall back to PIO. The ENEC after discovery exercises ring reuse after the terminal DAA NACK. No build or hardware validation was performed when adding this test.
