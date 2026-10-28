# I3C CCC test

Replaces `master_multi`. This firmware sends CCCs only; the application-data
test is now in `../mctp_write`. AST2700 kernel changes and a userspace receiver
are not required. Stop userspace transmitters while this suite enables and
disables SIR. Polling remains the default; the existing IRQ option is retained.

Target labels:
- `//target/ast10x0/tests/peripherals/i3c_hci/test_ccc:test_ccc`
- `//target/ast10x0/tests/peripherals/i3c_hci/test_ccc:test_ccc.elf`
- IRQ option: `--//target/ast10x0/tests/peripherals/i3c_hci/test_ccc:irq=true`

## Coverage

This covers the CCC command set currently defined in the Rust driver's
`ccc.rs`, with the prerequisites below. It is not a claim of coverage of every
command or format in every MIPI I3C revision.

| Commands | Verification / prerequisite |
| --- | --- |
| Broadcast RSTDAA, ENTDAA | Initial enumeration and final reset/rediscovery; compare count, PID, BCR, DCR |
| GETPID, GETBCR, GETDCR | Exact lengths and equality with ENTDAA identity |
| GETSTATUS | Exact two-byte response and logged raw status |
| GETMWL, GETMRL | Exact length; GETMRL includes IBI length when BCR advertises an IBI payload |
| Direct SETMWL, SETMRL | Write existing limits back and verify GET readback; does not prove a limit change is honored |
| Broadcast SETMWL, SETMRL | Same readback with one target; skip with multiple targets to preserve distinct limits |
| GETMXDS | When BCR advertises speed limitations; accept legacy 2- or 5-byte response |
| Direct ENEC/DISEC | SIR event bit for IBI-capable targets; bus completion only, not functional IBI delivery |
| Broadcast ENEC/DISEC | SIR event bit; command submission only, not functional IBI delivery |
| ENTAS0-3 broadcast and direct | Send each activity hint, return to AS0, check GETSTATUS responsiveness; not low-power/timing verification |
| SETNEWDA | Choose an unused dynamic address, update DAT, check identity, restore address and check identity again |
| Direct RSTDAA | Opt-in legacy command via `TEST_LEGACY_DIRECT_RSTDAA`; requires one target, then ENTDAA and identity verification |
| SETDASA | Set `TARGET_STATIC_ADDR` only if a static address really exists; requires one target; reset DA, assign via static address, verify identity |

SETDASA is skipped for the supplied AST2700 configuration because it has no
configured static address. GETMXDS is skipped for BCR 0x66 because bit 0 is clear.
Direct RSTDAA is disabled by default. Unsupported commands that are actually
attempted fail the suite; NACK and timeout are not silently counted as skips.

## Results and limits

The suite stops at the first transfer or data mismatch and prints
`TEST_RESULT:FAIL`. In particular, a successful GETPID transfer returning the
wrong PID now fails. `TEST_RESULT:PASS` means enabled checks passed; review
explicit SKIP messages for incomplete coverage.

Broadcast logs say SENT because broadcast descriptors may not return a response.
Address and length operations use readback where available. The controller
continues rejecting unsolicited IBIs through DAT while event CCCs are tested.
Tests do not cover MR/HJ event behavior, modern GETSTATUS/GETMXDS defining-byte
formats, HDR, timing control, debug/vendor CCCs, or controller-role handoff.
The timeout callback remains uncalibrated. If a transport failure interrupts
address restoration, rerun from reset; do not assume cleanup completed.

No build or board execution was performed when introducing this suite.
