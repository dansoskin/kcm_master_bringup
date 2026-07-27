# Plate Tilt Kinematics — Design

Date: 2026-07-27
Status: implemented; host tests pass, awaiting bring-up verification on hardware

## Problem

The machine carries a plate on a two-axis gimbal: two perpendicular motors sharing
one center of rotation. Today the plate is commanded per-motor, in motor degrees.
The operator wants to command it the way they think about it — "lean 10 degrees,
in that direction" — and have the firmware work out both motor angles.

## Mechanical model

The two axes are **co-located and decoupled**: both rotation axes are fixed in the
base frame and intersect at one point. Neither motor carries the other, so neither
motor's axis direction changes when the other moves. Each motor angle is therefore
an independent projection of the plate normal, not a link in a serial rotation chain.

This is the modelling assumption the whole design rests on. It must be checked at
bring-up (see Bring-up verification).

## Coordinate frame

- Z up. X and Y horizontal, perpendicular.
- At motor angles `(0, 0)` the plate is level and its normal is `+Z`.
- **Azimuth** `phi`: the compass direction the plate leans toward, measured from
  `+X`, counter-clockwise when looking down.
- **Tilt** `theta`: how far the plate leans away from level.
- Both inputs and both outputs are in **degrees**, matching the user units already
  established by `odrive_init(conv = 32.0/360)` — degrees at the output shaft.

At `theta = 0` the plate is level and `phi` has no effect; every azimuth maps to
the same `(0, 0)`.

## The math

Plate normal for a given pose:

```
n_x = sin(theta) * cos(phi)
n_y = sin(theta) * sin(phi)
n_z = cos(theta)
```

Each motor angle is the normal's projection into that motor's plane of rotation:

```
angle_about_X = atan2(-n_y, n_z)     /* leans the plate toward +/-Y */
angle_about_Y = atan2( n_x, n_z)     /* leans the plate toward +/-X */
```

Two notes:

**The sign asymmetry is intentional.** Under the right-hand rule, rotating about
`+X` by `a` sends the normal to `(0, -sin a, cos a)`, while rotating about `+Y` by
`b` sends it to `(+sin b, 0, cos b)`. One picks up a minus sign, the other does not.

**Use `atan2`, not `atan(tan(theta) * sin(phi))`.** The two are algebraically
identical, but `tan(theta)` diverges at 90 degrees and `atan2` resolves quadrants
for free. Same cost.

## Axis mapping

Confirmed with the user by behaviour, not by name:

| Motor  | Shaft lies along | Leans plate toward | Angle          | SyncAxis global |
|--------|------------------|--------------------|----------------|-----------------|
| `odrv0`| X                | +/- Y              | `atan2(-n_y, n_z)` | `y_axis`    |
| `odrv1`| Y                | +/- X              | `atan2( n_x, n_z)` | `x_axis`    |

The `SyncAxis` globals are named for the **lean direction**, so `x_axis` receives
the `n_x` term. That is the mnemonic to hold onto.

### Consequence for the existing `M` command

`M` currently binds `x_axis <-> odrv0` and `y_axis <-> odrv1`, which is the reverse
of the table above. `M` takes raw per-motor targets, so it is not *wrong* today —
but leaving it means `x_axis` would mean one motor in `M` and the other in `G`,
which is a bug waiting to happen. **Flip `M`'s mapping** to match the table as part
of this work.

## Sign conventions

Implemented as strict right-hand rule, with per-axis inversion available:

```c
#define PLATE_INVERT_TOWARD_X   (+1.0f)   /* scales out_x_deg -- odrv1 / x_axis */
#define PLATE_INVERT_TOWARD_Y   (+1.0f)   /* scales out_y_deg -- odrv0 / y_axis */
```

Named for **lean direction**, matching the `SyncAxis` globals and the output
parameters — not for the shaft each motor sits on. Naming them "about X / about Y"
would invert the association (odrv0's shaft is along X but it leans toward Y),
which is precisely the confusion this spec is trying to eliminate.

Actual motor direction depends on winding and mounting and gets settled with an
inclinometer at bring-up, not derived from the math.

These are deliberately *not* the `invert` flag on `odrive_init`. That flag would
also flip the raw `M` and `#A` commands; a kinematics-local sign keeps the change
scoped to the tilt interface.

## Travel limit

```c
#define PLATE_MAX_TILT_DEG   (30.0f)
```

Requested tilt beyond this is **rejected** — the function returns an error and
writes nothing to the output pointers, and the caller commands no motion. A typo
like `tilt=100` must not be able to drive the plate into a hard stop. 30 degrees is
the user-supplied figure; confirm against real mechanical travel at bring-up.

Azimuth is unbounded and wraps naturally — `sinf`/`cosf` handle any value, so
`phi = 450` behaves as `phi = 90`.

## API

New files `Core/Inc/plate_kinematics.h` and `Core/Src/plate_kinematics.c`.

Deliberately **not** placed in the `synchronized_movement` submodule: that submodule
is generic multi-axis pacing and is consumed by other projects, whereas this is one
machine's geometry. Keeping them apart keeps the submodule reusable.

```c
typedef enum {
    PLATE_OK = 0,
    PLATE_ERR_RANGE,    /* |tilt| > PLATE_MAX_TILT_DEG */
    PLATE_ERR_BAD_ARG   /* NULL out pointer, or non-finite input */
} plate_status_t;

/* Convert a plate pose to the two motor angles.
 *
 *   azimuth_deg  direction of lean, from +X, CCW looking down. Unbounded.
 *   tilt_deg     lean away from level. Rejected if > PLATE_MAX_TILT_DEG.
 *   out_x_deg    angle for the motor that leans toward X  (odrv1 / x_axis)
 *   out_y_deg    angle for the motor that leans toward Y  (odrv0 / y_axis)
 *
 * Pure: no hardware access, no globals. On any error the outputs are left
 * untouched, so a failed call cannot half-write a target. */
plate_status_t plate_angles_from_tilt(float azimuth_deg, float tilt_deg,
                                      float *out_x_deg, float *out_y_deg);
```

Pure and hardware-free, so it runs under the same host gcc harness already used to
validate the synchronized-movement solver.

## The `G` command

`G<azimuth>,<tilt>` in `Core/Src/decode_packet.c`, reusing the `count`/`result`
already split at the top of `decode_uart`:

1. `count < 2` → print usage, command nothing.
2. `plate_angles_from_tilt(...)` → on error print the reason, command nothing.
   Runs *before* any hardware check so a bad angle is rejected even when the
   drives are not ready.
3. Both `odrv0.pos_valid` and `odrv1.pos_valid` → else refuse. Same guard as `M`:
   `pos_estimate` is meaningless until an encoder frame has been decoded.
4. Read `current_position` from encoder feedback; write the computed
   `target_position` onto `x_axis` / `y_axis` per the mapping table.
5. `calculate_speeds_for_synchronized_movement(axes, 2)`.
6. Push each `sync_speed` as a traj vel limit, **then** `input_pos` — the ODrive
   plans its trapezoid when the position command arrives, so the limit must land
   first. Skip the limit for a zero-distance axis; 0 would stall its next move.
7. Echo the resulting pose and both motor targets.

Both motors therefore arrive together and the plate sweeps to the new pose in one
coordinated motion rather than two staggered tilts.

`M` keeps its raw per-motor interface (with the mapping flip noted above).

## Error handling

| Condition                  | Behaviour                                      |
|----------------------------|------------------------------------------------|
| fewer than 2 CSV fields    | usage message, no motion                        |
| tilt > `PLATE_MAX_TILT_DEG`| `PLATE_ERR_RANGE`, message, no motion           |
| non-finite azimuth or tilt | `PLATE_ERR_BAD_ARG`, message, no motion         |
| NULL output pointer        | `PLATE_ERR_BAD_ARG`                             |
| encoder feedback not ready | message naming both `pos_valid` flags, no motion|
| solver rejects the axes    | message, no motion                              |

Every failure path commands nothing. There is no partial-write state.

## Testing

Host test next to the existing solver test, same gcc harness:

- `(phi, 0)` for several `phi` → both outputs exactly 0 (level is level regardless
  of azimuth).
- `(0, 10)` → all of the lean on the toward-X motor, 0 on the other.
- `(90, 10)` → all of the lean on the toward-Y motor, 0 on the other.
- `(45, 10)` → both motors nonzero and equal in magnitude.
- All four azimuth quadrants → sign of each output matches the frame definition.
- Round-trip: recover `(phi, theta)` from the two motor angles and confirm it
  matches the input, for a sweep across azimuth and tilt.
- `phi = 450` behaves as `phi = 90` (wrap).
- `tilt = 31` and `tilt = 100` → `PLATE_ERR_RANGE`, outputs untouched.
- NaN and Inf inputs → `PLATE_ERR_BAD_ARG`, outputs untouched.

## Bring-up verification

The math is only as good as the mechanical model. On hardware, with an inclinometer:

1. `G0,10` — plate should lean 10 degrees toward `+X`, and only `odrv1` should move.
2. `G90,10` — 10 degrees toward `+Y`, only `odrv0` moves.
3. `G180,10` and `G270,10` — confirms both signs. Wrong direction on one axis means
   flipping that axis's `PLATE_INVERT_TOWARD_*` define.
4. `G45,10` — measured tilt magnitude should still be 10 degrees. If it reads high
   or low here while the pure-axis cases are correct, the co-located model is wrong
   for this mechanism and the serial-gimbal formulation is needed instead.
5. `G0,31` — must be refused.

Step 4 is the one that validates the modelling assumption; the others only validate
signs.

## Out of scope

Deliberately excluded, tracked separately:

- Inverse function (motor angles → current plate pose) for a status command. Cheap
  to add later; nothing needs it yet.
- Restoring each drive's `max_speed` traj vel limit after a synchronized move.
  Carried over from the earlier review as an open item.
- Enabling the cyclic encoder message so `pos_valid` becomes true. Also carried
  over; `G` will refuse to move until this is resolved.
- Checking the `odrive_status_t` returned by the CAN writes.
- The stepper as a third synchronized axis.
