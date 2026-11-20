# I3C HCI public interface

Application code uses `I3cController` or `I3cTarget`. Board bring-up still owns pins, clock/reset, cache/DMA mapping, PHY timing, role initialization and the core. Do not mix direct bus-management operations with an active application session.

`api.rs` defines the public traits, device identity and events. `controller.rs` implements the controller session; `target.rs` implements the target trait.

## Controller flow

1. Initialize a primary `I3cHciCore` with PIO or DMA selected and correct board timing.
2. Create `Controller::<_, N>::new(&mut core, scl_preset, yield_us)`. It applies the detected command format and initializes the selected transport. The mutable core borrow prevents another session being created through that core while this one is active.
3. Call `discover()`. The session owns its device table and DAT allocator. Repeated discovery preserves existing assignments and adds new targets.
4. Select a device with `device_by_pid`, `device_by_address`, or `devices()`.
5. Use `read`, `write`, `ccc_read`, `ccc_write`, `enable_ibi`, and `poll_event`. Applications never pass DAT indices or touch the ring.
6. Disable any enabled IBIs, then consume the session with `shutdown()`. Errors are returned explicitly; there is no fallible cleanup hidden in Drop.

For MCTP: enable IBI, wait for the target notification, validate MDB/optional PEC in the protocol layer, then promptly perform a private read. MCTP bytes and PEC policy do not belong in the HCI driver.

All three tests under `tests/peripherals/i3c_hci/mctp` use `I3cController`: `pio_write`, `pio_ibi_read`, and `dma_test`. Packet functions are generic over the trait; board setup selects PIO or DMA. The DMA test retains its diagnostic ENEC bypass: it logs an enable error and continues waiting for IBI. Normal driver response validation remains enabled. Low-level loopback tests remain unchanged so they can exercise descriptors, registers and transport details directly.

## Target flow

Board setup configures and enables `I3cHciTarget` with its chosen transport. Protocol code uses `I3cTarget` to queue data, queue/request an IBI, and poll completions. PIO and DMA IBI completions map to the same `TargetEvent::IbiComplete`. `ReadComplete.remaining` is the target TX response field, not the controller RX byte count.

Queue operations copy payloads into driver-owned DMA storage or hardware FIFO before returning. An IBI request is not completion; wait for the event before reusing the pending IBI operation. Queue order remains explicit, because the target's command FIFO order matters.

## Boundaries and current limits

- No IRQ/NVIC setup or kernel dependency is added. Polling can be called by bare-metal code today and a Pigweed task later. Transfers still wait synchronously with bounded driver polling; these traits are not an async scheduler or ISR API.
- A notification poll returns `None` when no event is ready; draining an event may perform bounded hardware polling. The controller poll preserves the caller's PIO signal-enable policy.
- DMA errors retain their diagnostic string in `I3cHciError::Dma`.
- Direct CCC address changes are rejected by the session because they require coordinated table updates. Use low-level bus management outside the session until typed address-change operations are provided.
- DMA payloads use variable-length, DWORD-aligned allocations from a 48 KiB reserved-SRAM pool per HCI. Controller transfers can use the available pool; concurrent target TX commands share it. Descriptor length and available contiguous SRAM bound each request, with no 64-byte per-transfer limit. The two pools plus descriptor/IBI storage fit the existing 128 KiB DMA region. IBI receive capacity remains independently bounded by its chunk ring. `with_timeout_us` follows the existing backend behavior; the DMA polling budget is currently fixed.
- A completed transfer releases its payload allocation. Failed/timed-out controller submissions retain their allocation because DMA may still reference it. Stop and reinitialize the ring before reusing those slots. Caller buffers may be reused when the synchronous API returns, since hardware uses driver-owned SRAM.
- The DMA IBI backend currently discards unsupported non-target-interrupt events. Hot-join notification coverage is therefore not equivalent between DMA and PIO; these traits do not claim otherwise.
- The rate argument selects a command preset; it does not synthesize an exact SCL clock. Board PHY timing must match it.
- Existing low-level APIs remain for diagnostics and incremental migration. This refactor establishes the application boundary; it does not remove all register/debug code from the backend.

No builds or hardware tests were run as part of this refactor.

## Internal responsibilities

- `master.rs` constructs controller commands and submits reads/writes through the selected transport. Immediate write payloads are handled in one submission helper.
- `target.rs` checks target state, dispatches shared command words to PIO or DMA, and classifies received writes/CCCs.
- `pio.rs` owns FIFO access. One drain helper handles word copying and padded tails; callers retain their existing overflow and error policies.
- `dma.rs` owns descriptor addresses, padded descriptor writes, circular IBI data copies and chunk reclamation. Controller and target status decoding remain separate because their layouts differ.
- `core.rs` owns initialization and register-clear polling. Vendor initialization runs once before bus enable.

DMA barriers, completion acknowledgements, and pointer publication remain explicit at their call sites. Sharing memory access does not combine the controller, target and DAA completion state machines.
