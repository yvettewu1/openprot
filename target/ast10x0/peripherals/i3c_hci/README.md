# I3C HCI public interface

Application code uses `I3cController` or `I3cTarget`. Board bring-up owns pins, clock/reset, cache/DMA mapping, role initialization and the core. Configured initialization calculates and programs PHY timing from the board settings. Use `I3cBusManagement` for topology changes so the active session retains correct address bookkeeping.

`api.rs` defines the public traits, device identity and events. `master.rs` implements the controller session; `target.rs` implements the target trait.

## Controller flow

1. Initialize a primary `I3cHciCore` with PIO or DMA selected and correct board timing.
2. Create `Controller::<_, N>::new(&mut core, scl_preset, yield_us)`. It applies the detected command format and initializes the selected transport. The mutable core borrow prevents another session being created through that core while this one is active.
3. Call `discover()`. The session owns its device table and DAT allocator. Repeated discovery preserves existing assignments and adds new targets.
4. Select a device with `device_by_pid`, `device_by_address`, or `devices()`.
5. Use `read`, `write`, `ccc_read`, `ccc_write`, `enable_ibi`, and `poll_event`. Applications never pass DAT indices or touch the ring.
6. Disable any enabled IBIs, then consume the session with `shutdown()`. Errors are returned explicitly; there is no fallible cleanup hidden in Drop.

For MCTP: enable IBI, wait for the target notification, validate MDB/optional PEC in the protocol layer, then promptly perform a private read. MCTP bytes and PEC policy do not belong in the HCI driver.

Hardware protocol tests under `tests/peripherals/i3c_hci` use the generic traits.
The two MCTP controller tests (`controller_pio` and `controller_dma`) use
`I3cController`. `mctp/target_dma` and `mctp/target_pio` share an
`I3cTarget` exchange with the chosen transport. Loopbacks use both traits; the CCC suite uses `I3cBusManagement`.
Board setup selects PIO/DMA and owns register diagnostics. Pure encoding and
FIFO-model tests intentionally exercise lower-level code. All four MCTP tests
echo the actual received bytes through the public traits, with protocol state
in the shared bare-metal helpers. The DMA controller test retains its ENEC
diagnostic bypass; normal driver response validation remains.

### Explicit bus management

`I3cBusManagement: I3cController` exposes non-address-changing broadcast CCCs,
`reset_addresses(None)` (broadcast RSTDAA), optional single-device legacy direct
RSTDAA, `set_address` (SETNEWDA), and `assign_static_address` (SETDASA for a known
unaddressed target). These operations belong to idle-bus setup/management, not
an active IBI exchange. Successful address changes update the session table;
use returned identities instead of old-address handles. SETDASA requires the
caller to know the target's actual static address and verified identity.

Generic raw CCC methods reject address-changing commands that would bypass
bookkeeping. After an uncertain address-changing failure the session enters RecoveryRequired: cached identities and ordinary traffic are blocked. A successful broadcast reset_addresses(None) restores known topology; if the command engine itself faulted, stop/reinitialize first. `read` and `ccc_read` reject responses clipped to the supplied buffer rather than
reporting a misleading length. Successful SETNEWDA moves the DAT entry and its
allocation to the new address; the old slot becomes available for later discovery.

`Controller::from_master` is for board lifecycle code that already owns a fresh,
undiscovered low-level master. It consumes that transport into a generic session.
Do not create multiple sessions over the same hardware. Normal applications
should use `from_configured_core`.

## Target flow

Board setup configures and enables `I3cHciTarget` with its chosen transport. Protocol code uses `I3cTarget` to queue data, queue/request an IBI, and poll completions. PIO and DMA IBI completions map to the same `TargetEvent::IbiComplete`. `ReadComplete.remaining` is the target TX response field, not the controller RX byte count.

Queue operations copy payloads into driver-owned DMA storage or hardware FIFO before returning. An IBI request is not completion; wait for the event before reusing the pending IBI operation. Queue order remains explicit, because the target's command FIFO order matters.

## Boundaries and current limits

- No IRQ/NVIC setup or kernel dependency is added. Polling can be called by bare-metal code today and a Pigweed task later. Transfers still wait synchronously with bounded driver polling; these traits are not an async scheduler or ISR API.
- A notification poll returns `None` when no event is ready; draining an event may perform bounded hardware polling. The controller poll preserves the caller's PIO signal-enable policy.
- DMA errors retain their diagnostic string in `I3cHciError::Dma`.
- Raw address-changing CCCs are rejected. Use I3cBusManagement so address changes and cached device state remain coordinated.
- DMA payloads use variable-length, DWORD-aligned allocations from the reserved-SRAM budget assigned to each enabled DMA instance in `config::BUSES`. Controller transfers can use the available pool; concurrent target TX commands share it. Descriptor length and available contiguous SRAM bound each request, with no 64-byte per-transfer limit. Configured pools plus descriptor/IBI storage must fit the existing 128 KiB DMA region; a compile-time assertion enforces the driver budget. IBI receive capacity remains independently bounded by its chunk ring. `with_poll_budget` controls unsuccessful probes per phase in both transports. These are iteration budgets, not elapsed microseconds. PIO retains its existing yield hook; DMA retains tight polling.
- A completed transfer releases its payload allocation. Failed/timed-out controller submissions retain their allocation because DMA may still reference it. Stop and reinitialize the ring before reusing those slots. Caller buffers may be reused when the synchronous API returns, since hardware uses driver-owned SRAM.
- PIO and DMA retain separate FIFO/chunk consumption rules. Shared controller IBI status decoding lives in cmd.rs.
- With `from_board()` / `init_configured()`, `scl_hz` selects the command bank and generates PHY counters. Explicit low-level constructors still require BSP-provided timing matching their command rate.
- Existing low-level APIs remain for diagnostics and incremental migration. This refactor establishes the application boundary; it does not remove all register/debug code from the backend.

No builds or hardware tests were run as part of this refactor.

## Internal responsibilities

- `master.rs` constructs controller commands and submits reads/writes through the selected transport. Immediate write payloads are handled in one submission helper.
- `target.rs` checks target state, dispatches shared command words to PIO or DMA, and classifies received writes/CCCs.
- `pio.rs` owns FIFO access. One drain helper handles word copying and padded tails; callers retain their existing overflow and error policies.
- `dma.rs` owns padded descriptor writes, circular IBI data copies and chunk reclamation; `dma_memory.rs` owns backing storage and address validation. Controller and target status decoding remain separate because their layouts differ.
- `core.rs` owns initialization and the successfully selected logical role. Vendor initialization runs once before bus enable.
- `completion.rs` owns bounded polling adapters. PIO/DMA own nonblocking progress, acknowledgement and completion; the four former `*_wait.rs` files are removed.

DMA barriers, completion acknowledgements, and pointer publication remain explicit at their call sites. Sharing memory access does not combine the controller, target and DAA completion state machines.

## Initialization role and wait boundary

DMA mode follows `core.init_mode()`: `init_primary()` selects controller transfers; `init_secondary()` and `init_target()` select target transfers. Plain `init()` does not choose a logical role, so DMA ring initialization rejects it. Role is recorded only after successful initialization. The bus number selects independent SRAM storage, never the role. Later controller-role handoff requires stopping/reconfiguring the ring; this change does not implement dynamic role switching.

Controller DMA follows descriptor publication → sampled event acknowledgement → DMA-idle probe → response-visibility probe → response dequeue. `progress_command` implements these phases; the shared polling adapter drives them. The wait helpers call one-shot readiness probes; the same probes and completion handling can be used by a future IRQ waiter. V2 DAA similarly separates pair publication, readiness observation, and completion/cancellation. PIO separates status readiness, waiting and response consumption. Target `poll_event` consumes an already-ready PIO response without entering the blocking response waiter.

Polling remains the only implemented wait policy. There is no IRQ-mode selector or new ISR/kernel integration yet. A Pigweed waiter must latch interrupt status before acknowledgement, avoid lost wakeups, preserve buffer lifetime through cancellation, and still check DMA SRAM visibility. It must not acknowledge the same event independently of the completion path. Public synchronous CCC/read/write APIs need not change when that waiter is added.

## Firmware bus configuration

`config.rs` is the firmware-owned board description, analogous to I3C nodes in DTS. Configure `BUSES[hardware_bus_number]` with:

- `enabled`, `role` (`Controller`, `Secondary`, or pure `Target`/slave), and default `io_mode` (`Pio` or `Dma`).
- Controller dynamic address, or target PID/BCR/DCR/static address through the role value.
- Requested SDR `scl_hz`, `TimingConfig`, data endianness and PIO timeout.
- Per-instance DMA TX/RX pool reservation, independent of selected transport; zero disables DMA and is required for disabled nodes.

The PAC contains slots 0 through 7. Enable only the seven instances actually present/available on your board, using their hardware numbers; an unavailable slot stays disabled. Defaults retain HCI0 controller and HCI1 secondary with DMA for existing diagnostics; other nodes are disabled. These are example firmware defaults, not fixed driver roles. Assign unique target identities appropriate to the board.

`CONTROLLER_BUS` and `TARGET_BUS` designate the board ports used by MCTP and loopback tests (HCI0 and HCI1 by default). Other enabled nodes do not affect that selection. Each test selects its PIO/DMA combination before initialization, retaining the board roles, identity and timing. No configuration edit is needed when switching tests.

Both Bazel driver libraries include `config.rs`. Edit that file and rebuild the application when ready. The former `dma_config.rs` aliases are removed; there is one source of configuration.

After BSP pin/clock/voltage/cache preparation, production code can initialize an enabled node and select its session:

```rust
let mut core = unsafe { I3cHciCore::from_board(bus_number, yield_us)? };
core.init_configured()?;
// For a Controller node:
let controller = Controller::<_, 8>::from_configured_core(&mut core, yield_us)?;
// For a Secondary or Target node instead:
let target = I3cHciTarget::from_configured_core(&mut core, yield_us)?;
```

`from_board()` uses the configured default transport. To select a different transport before initialization, use `I3cHciCore::from_board_with_io(bus_number, IoMode::Pio, yield_us)` (or `IoMode::Dma`). DMA selection requires a nonzero board reservation; PIO selection does not discard it. This constructor does not switch the transport of an active core.

The two session alternatives are mutually exclusive. The target constructor applies identity and preserves the configured Secondary/Target mode. Low-level constructors remain available to hardware tests with explicit initialization. DMA on those paths still requires an enabled DMA allocation in the firmware table.

DMA storage is compactly allocated for enabled nodes with a nonzero DMA reservation on any supported bus, independent of role and selected transport. Each instance has private descriptor storage, an IBI-data ring and an aligned TX/RX pool segment. Disabled and zero-budget nodes allocate none of these DMA resources. Selecting PIO on a DMA-capable node retains its static reservation, so another test image can select DMA without editing the board table. All seven board instances can be configured, but their combined DMA budgets must fit the board SRAM region (for example, seven 4 KiB payload budgets plus current ring storage fit 128 KiB). Descriptor strides remain hardware-reported and validated separately from payload budgets.

SCU/pinmux and physical wiring remain board responsibilities. DMA polling still uses its existing fixed budget. IRQ scheduling and secondary-to-active-controller handoff are not implemented by this configuration change.

### PHY timing

`BusConfig::timing` replaces the SDR4-copy workaround. `TimingConfig::automatic()` reads SCU300 HPLL parameters and SCU284's I3C divider without changing either register. BSP must finish clock setup first and keep that clock stable during operation. `input_clock_hz: Some(hz)` overrides detection for boards with a separately known PHY input clock; this is the peripheral input clock, not CPU Hz or desired SCL Hz. A disabled/reset PLL or an invalid rate is rejected rather than silently assuming a clock.

`scl_hz` requests 1..=12,500,000 Hz. Following ASPEED Zephyr's `vendor_aspeed.c`, initialization computes the SCL high/low counters from `ceil(input_hz / scl_hz) - 2`, uses the pure-I3C 40/60 split or mixed-bus 41 ns high-time calculation, and enforces the 60 ns minimum T-bit timing. Counter quantization means the physical rate can differ slightly from the request. It programs the selected SDR bank, SDR0 (including default/IBI timing), DDR timing, I2C FM/FMP defaults and open-drain/start/stop/hold timing before BUS_ENABLE. Programming these DDR counters does not add HDR transfer support. All 11-bit fields are range-checked before any PHY write.

`push_pull` and `open_drain` optionally provide `SclPeriods { high_ns, low_ns }`, equivalent to Zephyr's explicit DTS timing periods. Both periods must be nonzero. Explicit push-pull periods take precedence over the frequency-based counters; `scl_hz` still selects the command bank. `sda_tx_hold_ns: 0` selects the 10 ns default. `pure_i3c_bus` must be false when legacy I2C devices share the wires. `internal_pullup` is the ASPEED 0..7 encoding; zero leaves software pull-up override unchanged, matching Zephyr. PHY pull-up-enable initialization otherwise follows Zephyr.

For example, to retain a symmetric 1 MHz push-pull period instead of the calculated duty cycle:

```rust
scl_hz: 1_000_000,
timing: TimingConfig {
    push_pull: Some(SclPeriods { high_ns: 500, low_ns: 500 }),
    ..TimingConfig::automatic()
},
```

The default HCI0/HCI1 configuration uses automatic clock detection, a requested 1 MHz rate and calculated pure-I3C timing. Both ports use the same policy. DMA memory, descriptor/response ordering, IBI payload handling and the MCTP ENEC diagnostic bypass are independent of this timing setup.

Reference: [ASPEED Zephyr PHY timing](https://github.com/AspeedTech-BMC/zephyr/blob/aspeed-main-v3.7.0/drivers/i3c/mipi_hci/vendor_aspeed.c) and [AST10x0 G2 input clock calculation](https://github.com/AspeedTech-BMC/zephyr/blob/aspeed-main-v3.7.0/drivers/clock_control/clock_control_ast10x0_g2.c).

## Refactored responsibilities

- `master.rs`: application `Controller` session, device resolution, bus operations and a low-level executor retained for hardware diagnostics. `controller.rs` is removed. Repeated address/PID wrappers are replaced by resolving an `I3cDevice` through the session once.
- `cmd.rs`: pure V1/V2 encoding and rate selection. `ccc.rs`, `daa.rs`, `dat.rs`, and `dct.rs` remain separate. The device table now lives with DAT allocation.
- `transport.rs`: one selected PIO or DMA backend; attachment validates bus, role and controller format. Controller DMA is no longer restricted to HCI0. Target `with_dma` now returns `Result`.
- `completion.rs`: `Pending`, `Advanced`, `Complete` progress and bounded polling. Progress resets the per-stage budget; idle probes consume it. Budgets remain attempts rather than calibrated time. Target blocking DMA receive is now bounded too.
- `dma_memory.rs`: aligned storage, hardware-reported stride checks, address mapping and exclusive per-bus memory claims. Duplicate ring initialization is rejected before MMIO changes. Claims release on drop only after a confirmed stop; dropping a live or failed-to-stop ring retains its reservation. SRAM layout and identity bus mapping are unchanged.
- `registers.rs`: PAC access only, including raw DWORD access to dynamic table windows not represented as PAC arrays. `aspeed.rs` owns vendor setup sequences. `diagnostics.rs` owns opt-in register dump formatting.
- `error.rs`: typed DMA timeout versus fault classification; no message-string comparisons decide recovery.

Ordinary PIO private/CCC transfers and DMA commands use nonblocking progress functions under the synchronous polling adapter. V2 DAA retains its paired publication and special terminal-NACK cleanup. Target command encoding, IBI chunk reclamation, DMA barriers, ACK-before-response order, and target TX length interpretation are retained.

PIO controller event polling consumes only an available segment and does not wait for a later one. Keep the same output buffer until a complete event is returned. Both transports retain raw IBI metadata until controller event classification. Existing IRQ event latches can feed DMA completion without losing status already acknowledged by the ISR; there is still no Pigweed blocking waiter or new NVIC integration.

Call `shutdown()` for both controller and target sessions. Loopback cleanup now stops both DMA sides before releasing their storage. Explicit reset/recovery remains necessary after a failed shutdown; a successful response alone does not authorize reusing an outstanding DMA buffer.

The MCTP DMA test's ENEC diagnostic response bypass remains in the test. The production completion path still checks response visibility, TID and status. This refactor has been source-reviewed and formatted only: no compilation, tests or hardware runs were performed.

### Cleanup and ownership review

Sessions borrow their core/register lifetime; loopback callbacks return no borrowed
session. The ISR context owns an MMIO facade, not a reference to a stack session.
Board code enables NVIC delivery only after the transport and event subscription
are ready, and masks it before session shutdown. It then disables global signals
and unregisters its ISR context on success and returned failures.

Target shutdown always runs target disable, including a failed DMA stop. Global
signal/NVIC masking is still the board layer's responsibility. If target creation
fails and its cleanup also fails, the constructor reports that combined condition
instead of hiding the stop failure. Static DMA memory is released only after the
existing stop path confirms idle; a stop failure retains the reservation.

These guarantees cover returned errors. Panic/abort does not unwind or run cleanup.
No additional reset or FIFO/ACK sequence was introduced by this review.

## Scheduling and ownership

The internal Operation::advance method performs hardware progress without
scheduling. The Polling adapter owns the unsuccessful-probe budget and invokes
the PIO poll-yield hook. Regular controller commands, including CCCs, use guards
that borrow their engine exclusively; PIO guards also borrow caller buffers.
V2 paired DAA and hardware recovery keep their specialized sequences and use the
shared polling helpers. No kernel primitive or IRQ wait has been implemented yet.

For Pigweed, add a task-context wait adapter: register/arm the wakeup, recheck
progress to avoid a lost wakeup, then wait on the latched IRQ notification/deadline.
DMA idle and SRAM visibility phases may need a bounded recheck even without a new
interrupt. ISRs latch and mask I/O sources; deferred consumers own I/O W1C; they must not consume
responses, reclaim chunks or free payloads. Cancelled/failed regular command
guards block new submissions until reinitialization; live DMA storage stays
reserved. IBI draining remains available for the MCTP ENEC diagnostic bypass.

discard_partial_ibi() abandons a partially copied PIO notification. Repeated
calls drain at most one available segment each, without resetting shared FIFOs.
Once it returns true, a different output buffer may receive the next notification.
An untouched event is never discarded by this operation. DMA has not copied a
partial event, so it returns true and leaves that event queued in SRAM.

Controller and target sessions share transport shutdown. Callers still mask the
board/global interrupt path first; target disable runs even if DMA stop fails.
Explicit shutdown reports failures; Drop never releases storage still owned by
hardware. Raw constructors remain for board bring-up and diagnostics, with
exclusive hardware ownership required; applications use configured-core sessions.

daa.rs owns V1/V2 discovery orchestration; DAT/DCT modules remain separate.
diagnostics.rs owns ring dumps and the legacy expected-payload comparison helper.
registers.rs remains PAC access only. DMA timeouts retain their phase text in
I3cHciError::Dma(DmaError::Timeout(...)).

Regression cases cover pointer-field preservation/wrap, operation guard
cancellation, uncertain topology and partial-IBI discard. They were added but
not compiled or executed; validation for this change is source review and
formatting only.

## Refactor error-path review

PIO paired commands and commands without responses now retain the recovery
latch if a sequence fails after it starts. Oversized controller reads drain
their complete advertised RX payload before reporting NoSpace. DAA assignment
counts and write residual counts never cause controller RX FIFO reads.

An interrupted PIO IBI payload drain requires controller recovery: subsequent
poll/discard calls cannot reinterpret the uncertain payload tail as a status.
Normal partial-IBI discard remains nonblocking and preserves the next event.
The host FIFO-model cases cover these failures alongside the existing IBI cases.
These additional cases have not been built or executed.

## IRQ latch review

PIO FIFO readiness is sampled live. A latched RESP_READY, TX/RX threshold or IBI
threshold is only a wakeup hint; it cannot authorize FIFO access later.
Latched and live error bits are checked together before accepting readiness.
A response with a terminal DAA NACK still takes precedence over TRANSFER_ERR.

PIO/DMA ISRs latch status and save/mask I/O signal enables. They do not clear
I/O W1C bits: the deferred engine acknowledges those once at its existing
completion/IBI boundary. Snapshot and acknowledgement helpers serialize with
the ISR, and rearming restores saved subscriptions instead of replacing the
IBI mask with a command-only mask. Core IRQ acknowledgement remains in the ISR;
core/vendor latches remain available to integration diagnostics.

Target PIO polling uses the same latch-aware response path and rearms after RX
draining, including a fully drained NoSpace result. The raw PIO IBI-segment API
also restores IRQ delivery after successful draining or output overflow. I3cHciTarget::with_irq_events subscribes to a pre-registered bus latch,
matching the controller API. Registration rejects a mismatched bus/context.
Shutdown masks DMA ring signaling as well as core/PIO signals. Board code still
masks NVIC before shutdown and unregisters after the session stops.

Pigweed waiting/task wakeup integration is not implemented here. Its waiter
must arm the relevant sources and recheck progress before sleeping. After an
IRQ, idle/writeback/pointer visibility may require a bounded recheck without a
second IRQ. Neither kernel wakeups nor these regression cases have been tested
on hardware in this review.
