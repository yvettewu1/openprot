# I3C hardware test board configuration

Hardware tests use the dedicated `CONTROLLER_BUS` and `TARGET_BUS`
ports in [`config.rs`](../../../peripherals/i3c_hci/config.rs), defaulting to
HCI0 controller and HCI1 secondary target. Configure these ports once for the
board wiring. `BUSES` supplies roles, addresses, target identity, timing, timeout
and DMA memory reservations. Each test selects its transport at construction
through `from_board_with_io()`; switching test images needs no board-config edit.

## Test selection

| Test / target suffix | Runs on | Wiring / coverage |
| --- | --- | --- |
| `register_smoke:register_smoke` | AST1040 | No peer; configured controller register and section decoding |
| `driver_logic:driver_logic` | AST1040 | No peer or HCI setup; pure encoding, parsing, allocation and validation |
| `host/pio_ibi:pio_ibi` | Linux/x86-64 | No board; production PIO engine with FIFO model |
| `controller_ccc:controller_ccc` | AST1040 | PIO controller connected to an external target; real CCC suite |

These replace `i3c_init:i3c_hci`, `pio:pio`, `pio/host_ibi.rs`, and
`test_ccc:test_ccc`, respectively. Use the new labels; old aliases are not kept.
All firmware labels are under `//target/ast10x0/tests/peripherals/i3c_hci/` and
produce the named `.bin` and `.elf` through the shared runtime. The host test is
separate and marked `manual` so recursive firmware builds do not select it.

`driver_logic` intentionally uses pure low-level APIs to verify exact encoding.
`register_smoke` intentionally uses PAC-backed register access. Neither claims
to test actual bus traffic. Hardware protocol tests use `I3cController` and
`I3cTarget`; `controller_ccc` also uses `I3cBusManagement` for topology changes.

| Test | Controller transport | Local target transport | Completion delivery on AST1040 |
| --- | --- | --- | --- |
| [`mctp/controller_pio`](mctp/controller_pio/README.md) | PIO | External AST2700 | IBI, read, echo write; polling or optional `:irq=true` ISR latch |
| [`mctp/controller_dma`](mctp/controller_dma/README.md) | DMA | External AST2700 | IBI, read, echo write; polling with ENEC bypass |
| [`mctp/target_dma`](mctp/target_dma/README.md) | External AST2700 | DMA | Receive write, IBI, read echo; polling |
| [`mctp/target_pio`](mctp/target_pio/README.md) | External AST2700 | PIO | Receive write, IBI, read echo; polling |
| `loopback/pio` | PIO | PIO | Polling on both ports |
| `loopback/dma_pio` | DMA | PIO | Polling on both ports |
| `loopback/dma_dma` | DMA | DMA | Polling on both ports |
| `loopback/dma_v2` | DMA, V2-capable hardware | DMA | Polling on both ports |

MCTP package and target names now describe the AST1040 role:
`target_dma:target_dma`, `target_pio:target_pio`,
`controller_dma:controller_dma`, and `controller_pio:controller_pio`. The former `dma_target` and `dma_test`
were renamed; `pio_write` and `pio_ibi_read` were combined into
`controller_pio`. Old labels are removed. The optional IRQ flag is now
`--//target/ast10x0/tests/peripherals/i3c_hci/mctp/controller_pio:irq=true`.

The designated ports must be enabled with the matching controller/target roles.
Additional enabled buses do not affect test selection. `io_mode` remains the
board default for applications using `from_board()`; the tests explicitly select
the transports above before initialization, never while a bus is running.

Both default ports reserve 48 KiB of DMA payload memory. Reservations are
independent of the selected transport: PIO can use a port with reserved DMA
memory, and its reservation remains available to a DMA test image. A zero budget
disables DMA on that port. Budgets must be DWORD-aligned and fit the reserved SRAM
along with descriptors and IBI storage. Target BCR must advertise IBI with payload
for the loopback checks; discovery uses the board-configured target PID.

Loopback prints the selected HCI numbers: connect their SCL pins together and
their SDA pins together, with compatible voltage levels, isolated from AST2700.
MCTP controller tests use the selected controller wired to the external AST2700
target. `mctp/target_dma` and `mctp/target_pio` reverse the direction:
the dedicated AST1040 target port uses the selected transport and connects to
an external AST2700 master character device.

Pins, clocks, reset and cache policy remain in the shared bare-metal board setup,
which currently uses the existing LV routing and resets all HCI instances. These
are standalone tests: no other HCI traffic may be active. The board table's
`scl_hz` selects the command bank and calculates PHY counters during configured initialization. `timing` controls input-clock detection/override, duty-cycle overrides and SDA hold time. HCI0/HCI1 default to automatic SCU clock detection and a requested 1 MHz SDR rate; no SDR4-copy workaround is needed.
Loopback event polling budgets remain test-specific, and IRQ integration is not
added by this migration. The combined PIO controller test's optional IRQ routing follows the
selected hardware bus.

All four MCTP tests echo received bytes with their actual length and no fixed
payload comparison or injected header/data PEC. They run six exchanges and
print RX/TX data. The application buffers hold 128 bytes; nonempty packets must
also fit peer/tool/hardware limits. Controller tests wait for IBI, read, then
write the same data back. Target tests receive a write, queue the same data
with an IBI, then complete the master's read. The DMA controller's ENEC bypass
and existing IBI validation policies are retained. Loopback payload checks are
unchanged, and the V2 test still reports SKIP on V1-only hardware.

All loopback and MCTP tests use the shared `bare_metal/runtime.ld` through
the default in `bare_metal_image()`. It reserves `.dma_nc` at
`0x000a0000..0x000c0000`. Retained DMA storage stays out of ordinary SRAM/stack
space even when the selected transport is PIO. Change the shared script when
updating the test memory layout; per-test copies are not needed.

## Shared loopback lifecycle

`bare_metal/i3c_loopback.rs` owns dedicated-port selection, board reset/pin setup,
NVIC masking, timing logs, target/controller initialization, transport attachment,
discovery/ENEC setup, failure snapshots, shutdown and final result reporting for
all four loopbacks.
The V2 capability gate runs before either DMA ring is started. The target is
enabled before controller initialization and discovery.

Each loopback passes its controller transport, target transport and command format.
The callback uses generic controller/target traits. `discover_and_enable_ibi()`
retains the four-entry controller session, configured PID/address verification,
two-byte IBI limit and bounded DAA/ENEC notification drain. Both the ENEC
notification (`0x80`, payload `01`) and hardware SIR enable must be observed.
V2 uses its existing DMA discovery implementation without a PIO fallback.

V1/V2 DMA/DMA share `dma_rounds()`; PIO/PIO and DMA/PIO retain their distinct
working sequences. Six rounds, payloads, delays and completion checks remain.
The IBI completion status/TID mapping now belongs to `I3cTarget`; generic tests
check `TargetEvent::IbiComplete` instead of decoding raw target responses.
Setup and failure diagnostics may access registers, but protocol callbacks do
not submit raw descriptors, allocate DAT slots or poll DMA rings directly.

Cleanup snapshots loopback failures before stopping controller DMA, then stops
the target and masks both ports. Target cleanup also runs if controller setup
fails. Errors retain precedence over cleanup errors, with both failures logged.

`bare_metal/i3c_controller.rs` provides the board-configured lifecycle for the
external-target CCC and MCTP controller tests. It enables optional NVIC delivery only after the session is ready, masks it before
session shutdown, then masks global signals and unregisters its IRQ context on
every returned path. Session shutdown is attempted even after a protocol error.
`bare_metal/i3c_mctp_controller.rs` owns the shared six-round controller echo
protocol. It uses `I3cController` and the existing board/IRQ lifecycle; the
PIO test still attempts DISEC on exit, while the DMA diagnostic retains its
stop-only cleanup and ENEC bypass.

`bare_metal/i3c_mctp_target.rs` owns the target session, six-round MCTP exchange,
and equivalent cleanup for both `target_dma` and `target_pio`. Their entry
points select DMA or PIO explicitly. Both queue IBI then reply data before
requesting IBI, print/echo received bytes, and check IBI/read completion through
`I3cTarget`. Pending request bytes are kept separately from the RX scratch
buffer until queued, so intervening CCC events cannot corrupt the next echo. The PIO case uses no DMA ring. Returned exchange errors still
attempt shutdown; the original failure takes precedence over a logged stop
failure. Global signaling is disabled after session cleanup, with NVIC masked
throughout. A failed DMA stop retains its memory reservation.
Shared runtime vectors dispatch the existing optional IRQ tests for ports 0-7.
The optional IRQ path uses `register_i3c_hci_irq_handler`, session
`with_irq_events`, and `dispatch_i3c_hci_irq`. The ISR latches/masks; the driver
consumes responses and owns I/O W1C acknowledgement and signal rearming. Protocol
tests do not take/clear the latch or rearm signal masks themselves.

Polling sessions do not subscribe to the latch and leave NVIC masked throughout.
Status generation stays enabled so FIFO levels, DMA completions and IBI ring
pointers remain observable. Enabling peripheral signaling in the existing DMA
setup does not enable CPU interrupt delivery. Polling reads live hardware state;
no ISR is required. The loopbacks' `clear_hc_irq()` calls clear only core status
with NVIC masked, not PIO/DMA completion or IBI events.

Even with `:irq=true`, bare-metal transfers are driven by task-context polling;
this is ISR-latch coverage, not a Pigweed sleeping/wakeup test. Targets currently
use polling and do not require `I3cHciTarget::with_irq_events`. A future target IRQ
test must register its context, subscribe, then unmask NVIC, and mask it again
before shutdown. The combined PIO controller uses the existing optional ISR-latch path for the full exchange. The MCTP DMA
ENEC diagnostic bypass remains in that test's protocol code.

DMA initialization attempts to stop a ring if startup fails after enabling it.
Rejected consuming DMA attachments stop their incoming ring and any existing
owned ring before returning. A stop failure retains that ring's memory claim;
there is no implicit reset or reuse of possibly live DMA storage. The shared
runner logs controller/target shutdown errors even when an earlier test failure
is returned. Hardware that fails to stop may remain active: the log reports this
condition rather than claiming cleanup succeeded. These paths cover returned
errors; the bare-metal abort-on-panic handler does not unwind for cleanup.

## Loopback logging

Each initialized port prints two timing lines once, before discovery/transfers:
input clock Hz and its source (`SCU` or `override`), requested SCL Hz,
SCL Hz derived from the counters, SDR bank, and calculated SCL/T-bit/hold/OD words.
`H` and `L` are the encoded high/low counts; each represents count + 1 clocks.
The values are saved from successful initialization, not recalculated at log time.
SDR0 readback is also checked and printed only if it differs. `counter_scl` is
calculated, not a measurement of the wires; the target port's values describe
its PHY configuration, while the active controller supplies SCL.

Normal loopback output retains bus selection, target discovery, phase PASS lines,
and TEST_RESULT. Startup register tables, descriptor/response traces, per-byte
payload dumps and repeated PHY/queue snapshots are suppressed. Failures print
non-destructive state snapshots for both ports before shutdown. Other tests keep
the existing `prepare_and_show()` diagnostic setup.

## Review checks

Generic private and CCC reads reject clipped responses. Both PIO-target loopbacks
now require zero remaining target TX bytes, as the DMA-target checks already did.
Bounded event helpers retain the original minimum of one probe for a zero budget.
Successful SETNEWDA moves both the driver's cached identity and DAT allocation,
so reset/restore and subsequent discovery do not retain an occupied old slot.

The DMA/PIO transfer engines, ring ACK/dequeue ordering, payload/IBI sequences,
read delays and DMA MCTP ENEC bypass were compared by source during review.
Formatting/source checks are not a substitute for compilation or board validation;
no build or hardware execution was performed for this review.


The IRQ API review also adds host-model cases for six write/IBI/read rounds in
polling and ISR-latch modes, unsubscribed stale-latch isolation, and target PIO
short-RX/overflow/error handling. These cases are source-reviewed only, not built
or executed; they do not validate DMA hardware or actual IRQ timing.
