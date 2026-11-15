# Experimental rolling Restart Assist

This selectable balance-control policy is disabled for LIVE. It uses the same output
arbiter, acknowledged-zero reversal guard, sensor checks and permanent disarm path.
It changes neither PD gains nor ordinary PWM mapping. ACTIVE remains 90 seconds.
The fixed lifetime two-pulse policy remains selectable with
`balance-control-restart-assist`.

| Parameter | Rolling policy |
|---|---:|
| Window | 1.5 seconds |
| Pulse count | 2, shared across directions |
| Assist-on budget | 300 ms per window |
| Individual pulse maximum | 150 ms |
| Duty ceiling | 70% |
| Cooldown after acknowledged pulse end | 300 ms |
| Acknowledged zero interval | 100 ms |
| Fresh stable demand dwell | 20 ms |

The existing 60 ms scheduler/IO stopping reserve is preserved. An ideal 10 ms replay
therefore ends a pulse at 90 ms; 150 ms is a maximum, not a promised duration.
Neutral demand, conflict, sensor/timing faults and disarm still cancel immediately.
No command implies measured wheel speed or successful movement.

## Ledger and recovery

A fixed two-entry ledger records accepted assist start and acknowledged end times.
Active entries never expire. Completed entries remain until **end + 1.5 seconds**;
expiry is inclusive at that timestamp. Full retained durations are charged rather
than only their overlap with the trailing window. This conservative rule can delay
recovery, but never grants extra credit at a partial-overlap boundary. Timestamp
overflow retains the entry. A full ledger rejects a new entry; it cannot overwrite
an unexpired pulse. An unexpected acknowledgement/accounting failure latches a fault.

Every pulse overlapping the trailing window is retained, and its charged duration
is at least the overlap. Thus the ledger's two-entry and 300 ms limits also bound
every trailing 1.5-second interval, subject to the existing scheduler/IO bounds.
Those bounds are tested with independent clipped-interval accounting, including
pulse endpoints, expiry endpoints and +/-1 us boundaries. Software cannot guarantee
pulse termination during a CPU hang; independent motor-power cutoff remains required.

Neutral, direction changes, cooldown and controller state changes do not clear the
ledger. Disarm is permanent. Credit recovery only restores eligibility. A meaningful
request denied during cooldown or exhaustion is consumed, including during reversal
waiting: holding it cannot produce a delayed assist. A new fresh neutral-to-demand
or meaningful sign-change request must pass all guards, zero acknowledgement and
stable-demand dwell. Opposite polarity still waits 100 ms after acknowledged zero.
Initial and subsequent assists use the same ledger; no separate mapper Kick budget
is active in this mode.

## Selection and telemetry

Select `balance-control-rolling-assist` separately from the fixed policy feature.
`ROLLING_ASSIST_REVIEWED` in `src/bin/balance_control_test.rs` remains **false**.
Both policy features together, or an unreviewed selection, force DRY mode. The
existing operator verification values are preserved; no new LIVE permission is
granted. Reviewing this policy does not itself verify motor power or restraint.

Fixed-size recorder entries include proposed, approved and acknowledged commanded
PWM, suppression reason, lifetime assist time, remaining window count/time, both
retained intervals, earliest recovery timestamp/delay, cooldown and fresh-request
requirement. Serial printing occurs after permanent shutdown. Recorder saturation
is counted and first/last applied and terminal records are retained.

Safe host replay (no device access):

```sh
CARGO_ENCODED_RUSTFLAGS='' cargo +stable run --offline --locked \
  --manifest-path tests/orientation/Cargo.toml --target x86_64-unknown-linux-gnu \
  --example balance-control-replay -- --rolling-assist
```

Firmware build only (does not flash):

```sh
cargo build --offline --release --locked --bin balance-control-test \
  --features balance-control-test,balance-control-rolling-assist
```

Two pulses and 300 ms per 1.5 seconds bound assist command time to 20%; they do
not bound electrical energy, temperature or ordinary stalled-motor current. Supply,
loaded response, mechanical restraint and independent power cutoff still need
hardware verification. This is neither autonomous balancing validation nor permission
to run unrestrained. Window recovery intentionally permits recurring experimental
assistance; the configured ACTIVE timeout remains the outer limit.

A separate, default-off [pre-cutoff recovery experiment](pre_cutoff_recovery.md)
can qualify a bounded same-demand retry per observed deterioration episode. Without that additional feature the original
fresh-request requirement remains unchanged.
