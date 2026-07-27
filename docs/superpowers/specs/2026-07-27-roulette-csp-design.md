# Roulette — Stepper-Mastered Conical Wobble via CSP

Date: 2026-07-27
Status: implemented; host tests pass, awaiting bring-up verification on hardware
Depends on: [plate tilt kinematics](2026-07-27-plate-tilt-kinematics-design.md)

## Goal

The physical stepper acts as a master azimuth axis. The two ODrives follow it at
200 Hz with streamed position setpoints (CSP), so the plate holds a constant tilt
while its high point orbits — a conical wobble.

A spin is a **bounded sequence**: `R<degrees>` gives the total azimuth to travel,
and the stepper's own trapezoid handles ramp up, cruise and ramp down into a
stop. `R360` is one orbit, `R3600` is ten; a negative value reverses.

Tilt, speed and total live as globals in `roulette_sm.c`
(`roulette_tilt_deg`, `roulette_speed_deg_per_sec`, `roulette_total_deg`). All
are read once when the spin starts and fixed for the run — nothing changes on
the fly.

## Units

`FlexyStepper_setConversion(&stepper, 3200.0f / 360.0f)` — 8.889 steps per degree
of azimuth. One motor revolution (3200 steps) is exactly one plate orbit, giving
0.1125 deg of azimuth per step.

Motion limits, from the requirement "90 deg/s reached after one rotation":

```
a = v^2 / 2d = 90^2 / (2 * 360) = 11.25 deg/s^2   -> 8 s spin-up
```

**Consequence, accepted:** the `@` stepper commands previously spoke revolutions
and now speak degrees. `@R10` moves 10 degrees, not 10 turns.

## Why PASSTHROUGH, not TRAP_TRAJ

In `TRAP_TRAJ` every `input_pos` write makes the drive plan a whole trapezoidal
move ending at zero velocity. Streaming that at 200 Hz means replanning a
decel-to-stop toward a target 0.9 deg away, every 5 ms — the drive spends its life
braking toward a point just ahead of itself. `TRAP_TRAJ` means "here is a
destination, take your time"; streaming means "this is where you should be right
now". `PASSTHROUGH` means the latter.

The cost is that `PASSTHROUGH` removes the drive's ramp safety net: a bad setpoint
is a step command. `start_roulette_sm()` is the only gate between the globals and
the stream, so it range-checks all three and refuses with a specific reason
rather than starting.

## Why velocity feedforward

The position loop is `vel_setpoint = pos_gain * error + vel_ff`. Without `vel_ff`
the only way the motor moves at velocity v is to hold an error of `v / pos_gain` —
the error *is* the command, so a proportional controller must lag a moving target.

Modelled as first order with `pos_gain ~= 20` (ODrive default; confirm on the
bench), the lag appears as a phase shift of the whole wobble:

| Spin speed | Wobble rate | Azimuth phase lag | Tilt for 10 deg commanded |
|------------|-------------|-------------------|---------------------------|
| 90 deg/s   | 1.57 rad/s  | 4.5 deg           | 9.97 deg                  |
| 180 deg/s  | 3.14 rad/s  | 8.9 deg           | 9.88 deg                  |
| 360 deg/s  | 6.28 rad/s  | 17.4 deg          | 9.54 deg                  |

Amplitude barely suffers; the plate's high point trails the commanded azimuth, and
the faster it spins the further it trails. Feedforward supplies the correct
velocity at zero error, so the proportional term only corrects real disturbances.

Headroom is ample: the library packs `vel_ff` as int16 milli-turns/s
(`odrive_setpoints.c:20`), giving +/-368 deg/s in user units at 0.011 deg/s
resolution, against a peak motor rate of ~16 deg/s at 10 deg tilt and 90 deg/s.

The feedforward is computed analytically by differentiating the kinematics. With
tilt constant and azimuth advancing at rate w:

```
x_dot = -w * tan(tilt) * sin(az) * cos^2(x)
y_dot = -w * tan(tilt) * cos(az) * cos^2(y)
```

Both factors are dimensionless, so w in deg/s yields rates in deg/s directly.

This lives in `plate_kinematics` as `plate_rates_from_spin()`, **not** in the
roulette module — otherwise two modules would encode the gimbal geometry. It also
makes it testable: the host test numerically differentiates
`plate_angles_from_tilt` and confirms the analytic rate matches.

## Driver constraints that shape the design

Three findings in cFlexyStepper drive the approach:

1. **`jog()` zeroes the position** (`cFlexyStepper_Basic.c:51`), which would snap
   the plate to azimuth 0 the moment a spin began.
2. **`jog(0)` is an E-stop, not a ramp** (`cFlexyStepper_Basic.c:102`) — sets
   target to current and zeros the step periods. The master halts in one tick.
3. Therefore **`jog()` is not used at all.** The spin is `setSpeed(v)` plus a
   single `setTargetPositionRelative(total)`, so the driver's own trapezoid
   produces the entire ramp-up / cruise / ramp-down profile. An early abort uses
   `setTargetPositionToStop()`.

## State machine

Follows the project template (`set_*_state` logging via `send_uart`, parallel
string table, `HAL_GetTick()` for `millis()`), modelled on the driver's homing SM.

```
ROULETTE_IDLE          drives in TRAP_TRAJ, nothing streaming
ROULETTE_ARMING        logging muted, PASSTHROUGH frames sent, brief settle
ROULETTE_ACCELERATING  streaming; master ramping toward cruise speed
ROULETTE_SPINNING      streaming; cruise, plus the trapezoid's own ramp-down
ROULETTE_STOPPING      aborted early, still streaming, waiting on motionComplete
ROULETTE_FAULT         a drive left closed-loop control
```

`ARMING` exists so the `PASSTHROUGH` mode-switch frames land before the first
setpoint goes out; the relative move is issued when it expires.

The sequence ends by itself — `motionComplete()` is checked from `ACCELERATING`,
`SPINNING` and `STOPPING`, and whichever sees it first runs teardown and returns
to `IDLE`. Checking it from `ACCELERATING` matters: a short total such as `R30`
finishes before cruise speed is ever reached.

`STOPPING` is only for an early abort (`R0` or `K`). Deceleration takes seconds,
so the drives cannot be put back into `TRAP_TRAJ` at the moment the abort is
typed — the stream is still running.

The tail-end deceleration of a normal sequence happens inside `SPINNING` rather
than `STOPPING`: the transition to `ACCELERATING` is one-way, so a falling
velocity near the end does not bounce the state back.

### Teardown (leaving STOPPING, or on fault)

Restore `TRAP_TRAJ`, restore each drive's `max_speed` traj vel limit, unmute
logging. The vel-limit restore was previously an optional cleanup item; it is
mandatory here, because otherwise a `G` after a spin inherits a stale limit.

### Logging must be muted while streaming

`odrive_set_input_pos` calls `odrive_logf` on every invocation. At 200 Hz across
two drives that is 400 UART lines per second, which would drown the console and
starve the main loop. `odrive_enable_logging(od, 0)` on arm, restored on teardown.

## Precision over a long run

Two separate concerns, easily confused:

- The **stream** wraps the azimuth with `fmodf(pos, 360)` every tick. That is
  about feeding the kinematics a sane angle.
- **Start** reduces the master's own position with the same `fmodf`. That is
  purely about magnitude: position carries over between sequences, and float32
  degrees lose resolution as the number grows — roughly 0.06 deg once it reaches
  1e6, which is a few thousand `R360` runs. Dropping whole turns preserves the
  azimuth phase, so the bookkeeping changes and the plate does not.

This is defensive rather than required; a power cycle resets the count anyway.
It runs once per start, not per tick.

## Commands

- `R<degrees>` — run one sequence of that much azimuth, then settle. `R360` is
  one orbit, `R3600` is ten, negative reverses. Refused, with the reason printed,
  if the tilt is out of range, the speed is not positive, the encoders are not
  yet reporting, or either drive is not in closed loop.
- `R0` — abort a running sequence.

Tilt and speed are set by writing `roulette_tilt_deg` and
`roulette_speed_deg_per_sec` (debugger, or a future command). Neither has any
effect mid-sequence.

## Accepted risks

An explicit operator decision, recorded so it is not mistaken for an oversight:

- **No busy-guard on the sync moves during a spin.** They will fight the tick for
  the same drives, and the tick — writing every 5 ms — wins.

Straightforward to add later.

## Testing

Host tests for `plate_rates_from_spin` against central-difference differentiation
of `plate_angles_from_tilt`, swept over azimuth and tilt, plus sign, zero-rate and
error-path cases. The roulette module itself touches HAL, CAN and timing, so it is
verified on the bench rather than in the host harness.
