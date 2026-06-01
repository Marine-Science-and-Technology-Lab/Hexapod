# MaST Hexapod GUI — Functional Model

A functional description of the MATLAB application that monitors and controls
the IIHR Stewart-platform hexapod through a Galil DMC-40x0 motion controller
(reached over Ethernet via the `gclib` Python library, with a software emulator
stand-in for hardware-free testing).

> Scope: this document models *what the system does and how data flows through
> it*, derived from the source. For the planned/pending work and App-Designer
> wiring tasks, see [UPGRADE_PLAN.md](UPGRADE_PLAN.md) and
> [APP_DESIGNER_WIRING.md](APP_DESIGNER_WIRING.md).

---

## 1. System purpose & physical stack

The hexapod is a 6-DOF Stewart platform: a moving platform connected to a fixed
base by **6 linear actuators**, each ending in **U-joint yokes** (the AB joint on
the base, the CD joint on the platform). A test article ("end effector" / POI)
mounts on the platform. The GUI lets an operator:

- jog/position the platform in 6 DOF (X, Y, Z, roll, pitch, yaw),
- plan complex motion profiles (point-to-point, sinusoidal, multisine, imported),
- **simulate** a profile (kinematics + actuator dynamics) and check it against
  mechanical/electrical limits before running,
- **execute** it by streaming contour data to the Galil controller,
- watch a real-time **digital shadow** (where the hardware *actually* is, solved
  from encoder feedback), and
- measure platform pose optically from a checkerboard target (Basler camera).

```
            Operator
               │
        ┌──────▼───────┐   MATLAB App Designer GUI
        │  HexControl  │   (HexControl.mlapp)
        └──────┬───────┘
               │ app.g.GCommand / GProgramDownload  (gclib surface)
        ┌──────▼───────┐
        │   gclib  py  │  ── OR ──  GalilEmulator (software)
        │ (Python via  │
        │  MATLAB py.) │
        └──────┬───────┘
               │ Ethernet (192.168.42.2)
        ┌──────▼───────┐
        │ Galil DMC-40x0│  axes A B C _ E F G  (D unused)
        └──────┬───────┘
               │ servo amps + encoders
        ┌──────▼───────┐
        │  6 actuators │ → Stewart platform → test article
        └──────────────┘
```

The control axes are the Galil letters **A, B, C, E, F, G** (six actuators); axis
**D is intentionally skipped**. This shows up everywhere as the `,,` gap in
`CD a,b,c,,d,e,f` contour strings and as `ACTIVE_SLOTS = [1 2 3 5 6 7]`.

---

## 2. Software architecture (layers)

| Layer | Responsibility | Key files |
|---|---|---|
| **GUI / orchestration** | Tabbed control panel, callbacks, lamps, DROs, timers | `HexControl.mlapp` (main), `Point_to_Point.mlapp`, `SineSeries.mlapp`, `Hexapod_Generate_MultiSine_Excitation.mlapp` |
| **Motion planning** | Turn user intent into a pose trajectory `hex_path.pose_t` | `JogFromCurrentPosition`, `ReturnToDatumPath`, `squashfunction`, `Grab_Point_to_PointData`/`Grab_SineSeriesData` (in app), `Convert_multisine_to_hex_path` |
| **Simulation / verification** | Kinematics + dynamics over the whole trajectory; limit checks | `SimulateMotionProfile_kinematics`, `SimulateMotionProfile_dynamics`, `Plot_Check_Motion`, `FindEdges` |
| **Kinematics core** | Single-pose IK and FK, frame algebra | `InverseKinematics_hexapod`, `ForwardKinematics_hexapod`, `composeTransform`/`invertTransform`/`poseToTransform`/`transformToPose`, `E2R`/`R2E`/`AA2R`/`skew`, `EffectorFromPlatformPose`, `LengthToEncoder`/`EncoderToLength`/`dist2counts` |
| **Controller I/O** | Stream contour data, soft-stop, buffer health, trigger pulses | `ContourStreamSession` (async timer engine), `StreamContourData_to_Galil` (sync wrapper), `StreamContourData_to_Galil_w_trigger`, `CancelToken` |
| **Controller abstraction** | One `gclib`-compatible surface for real or fake hardware | `galilFactory`, `Emulator/GalilEmulator` |
| **Live feedback** | Poll encoders → solve FK → update DROs/gauges at 10 Hz | `updateDigitalShadow`, `updateEncoderDROs`, `updateGauges` |
| **3-D graphics** | Render & animate platform geometry | `hexapodGraphic`, `hexapodGraphic_nocalc_replot`, `initialize3Dplot`, `AnimateHexapod` |
| **Camera / metrology** | Optical pose from checkerboard, recording | `GrabCheckerboard`, `MeasurePoseFromCheckerboard`, `FindEdges`, `Camera_Recording`, `TestCheckerboardRealTime`, `Platform2ModelCM` |
| **Collision (offline)** | STL-mesh U-joint collision study | `CollisionMeshes/Code/*` (standalone demos) |
| **Setup / config** | Build the hexapod object & parameters | `InitializeHexapodObject` |

---

## 3. Core data structures

Three structs flow through almost every function.

### `hex_obj` — the hexapod state object (built by `InitializeHexapodObject`)
- **Fixed geometry:** `base` (3×6 base joint positions), `plat0` (3×6 platform
  joint positions, reordered so links connect correctly), `base_link`/`plat_link_0`
  (joint points offset by yoke Z-heights), `L0` (min length), `dL` (stroke).
- **Frame transforms (the three-frame model, see §5):**
  `T_world_datum_platform` `{R,t}`, `T_platform_POI` `{R,t}`,
  `Home_platform` (immutable 3×1 re-home position).
- **Live pose:** `pose` = `[x;y;z;roll;pitch;yaw]` = the POI pose in the **datum**
  frame (the motion command); `pose_platform` = platform pose in world (filled by IK).
- **Derived (written by IK):** `axisPos` (6 link lengths, m), `axisCt` (encoder
  counts), `plati`/`plat_link_i` (world joint positions), `joint_AB`/`joint_CD`
  angles, `joint_separation.AB/.CD` (mm clearance).

### `hex_setup` — machine parameters (built by `InitializeHexapodObject`)
- `Motors.*` (Kt, torque/RPM/current ratings, `CountsPerRev`),
- `Actuators.*` (`CountsPerM`, max speed/accel, `MinLength`/`MaxLength`,
  `DatumLength_Individual` per-axis),
- `Platform.*` / `Effector.*` mass & inertia, `F_ex.*` external force/moment,
- `Joint_Interp.SCAT_AB`/`SCAT_CD` (scattered interpolants from
  `JointSep_Interpolants.mat` mapping U-joint angles → yoke separation),
- `collisionthreshold` (min allowable yoke clearance, default 0.5).

### `hex_path` — the motion profile (the thing that gets executed)
- **Input:** `T` (time vector), `dt` (timestep, default `2^2/1024` s ≈ 3.9 ms),
  `pose_t` (6×N pose trajectory, treated as *relative* — Simulate adds `hex_obj.pose`).
- **After kinematics:** `pose_t_relative`, `pose_dt`/`pose_ddt`, `axis_t` (link
  lengths 6×N), `axis_dt`/`axis_ddt`, `axis_cts` (encoder counts 6×N), `plati`,
  `lhat`, `joint_AB`/`joint_CD`, `joint_separation`, `collisioncheck`.
- **After dynamics:** `F_links`, `FT_platform`, `MotorTorques`, `MotorCurrent`,
  `MotorRPM`.

---

## 4. The GUI (HexControl) — control surface

`startupFcn` builds `hex_obj`/`hex_setup`, loads `JointSep_Interpolants.mat`,
adds the gclib Python path, runs one IK + render, creates a `CancelToken`, and
wires the `ActuatorPositions` cell array (30 DRO widgets, see below). Tabs:

- **Start & Home** — Initialize Controller, Re-Home Actuators (`HM`), Lock/Unlock
  brakes (`MO`/`SH`), jog buttons (X/Y/Z ± and Rx/Ry/Rz ±), Return to Datum,
  Set Datum Here / Reset Datum to Home, jog-distance & angle-increment fields.
- **Test Article Set-Up** — sub-tabs:
  - *Coordinate System*: X/Y/Z mm offset → `T_platform_POI.t` (where the POI sits
    on the platform).
  - *Mass Distribution*: total mass, CG, inertias → `hex_setup`.
  - *External Force*: Fx..Mz → `hex_setup.F_ex`.
- **Plan Motion** — generate (Point-to-Point, Sinusoidal), Import/Export `.mat`,
  Simulate, Animate, Execute Motion, **Soft Stop**.
- **Diagnostics** — *Emulate Galil Connection* checkbox, save app vars to workspace.
- **Camera** — Measure Pose (checkerboard), Get Current Platform Position,
  Home & Generate Path, image axes `UIAxes3`.

**Status lamps** (`UpdateLEDs`): ControllerInitialized, PositionKnown, AtHome,
MotionProfileOK, ReadyToRun, AllLimitSwitchesClear, EStopStatus — green when the
matching `LED_stat` field is 1, red when 0 (interpolated for in-between).

**DRO panel** — `ActuatorPositions{1..30}`: 1–6 linear gauges (actuator length m),
7–12 encoder counts, 13–15 X/Y/Z (m), 16–18 roll/pitch/yaw (deg), 19–24 AB joint
clearance, 25–30 CD joint clearance. Written by `updateGauges` (commanded) and
`updateDigitalShadow` (measured).

**Child apps** call back into HexControl: `Point_to_Point` builds a segment table
then calls `Grab_Point_to_PointData(callingApp, table, type, datum)`; `SineSeries`
calls `Grab_SineSeriesData(...)`. Both populate `app.hex_path`.

---

## 5. Coordinate-frame model

Four frames, composed via homogeneous transforms (`{R,t}` structs):

```
World (W) ──T_world_datum_platform──▶ Platform-at-pose-0
   │                                        │
   │                              T_platform_POI
   │                                        ▼
   └────────── T_world_datum ─────────▶ Datum (D) origin == POI at pose=0
                                            │
                              pose = T_datum_POI = [x y z roll pitch yaw]
                                            ▼
                                          POI (Q)  (the commanded point)
```

- **`pose`** (the motion command) is the POI's pose **relative to the datum**.
- IK composes the full chain to get the **platform** world pose, then solves link
  lengths:
  `T_world_POI = T_world_datum · T_datum_POI`,
  `T_world_plat = T_world_POI · inv(T_platform_POI)`.
- Euler convention (`E2R`): body-fixed X→Y→Z, `R = Rz·Ry·Rx`.
- User controls move frames, not geometry: *Coordinate System* tab sets
  `T_platform_POI.t`; *Set Datum Here* captures current platform pose as the new
  datum; *Reset Datum to Home* re-anchors the datum at `Home_platform`.
- At default init both transforms are identity-rotation, so output matches the
  pre-refactor single-frame behavior exactly.

---

## 6. Primary functional flows

### 6.1 Connect to controller
`InitializeControllerButton` → `galilFactory(Emulated, '192.168.42.2')` returns
either `py.gclib.py()` (real, `GOpen` to the IP) or a `GalilEmulator` (opened at
sentinel `'emulator'`). `GInfo` string is shown, ControllerInitialized lamp goes
green, `Connected=1`, and a **10 Hz digital-shadow timer** is started. The
Emulate checkbox just flips `app.Emulated` and forces a re-Initialize.

### 6.2 Jog (and Return-to-Datum)  — the canonical command pipeline
```
jog button ──▶ JogFromCurrentPosition  (2 s squashfunction S-curve, relative pose_t)
           ──▶ SimulateMotionProfile_kinematics
                   pose_t(+hex_obj.pose) ─▶ batched IK over N steps
                   ─▶ axis_t (lengths), axis_cts (counts), joint_separation,
                      collisioncheck
           ──▶ if collision predicted → msgbox, abort
           ──▶ softStopToken.reset(); StreamContourData_to_Galil(g, hex_path, …)
           ──▶ update hex_obj.pose to endpoint, re-IK, replot 3-D, updateGauges
```
Return-to-Datum is the same pipeline with `ReturnToDatumPath` (S-curve to the
datum origin) instead of `JogFromCurrentPosition`, and it clears the soft-stop
lockout on success.

### 6.3 Plan → Simulate → Execute (a full profile)
```
Generate / Import  ─▶ hex_path.pose_t, T, dt
Simulate Motion    ─▶ SimulateMotionProfile_kinematics  (lengths, counts, clearances)
                   ─▶ SimulateMotionProfile_dynamics    (F_links, torques, current, RPM)
                   ─▶ Plot_Check_Motion → Pass?  (length/vel/accel/force/RPM/torque/
                                                   current/collision limits)
                      Pass ⇒ enable Execute + Export
Execute Motion     ─▶ guard softStopLockout
                   ─▶ StreamContourData_to_Galil(g, hex_path, trigger=1, …)
                   ─▶ update displayed pose, re-IK, replot, gauges
```

### 6.4 Streaming to the Galil (`ContourStreamSession`)
A handle-class **state machine on a MATLAB timer** (`TICK_PERIOD` 50 ms), so the
UI stays responsive. `StreamContourData_to_Galil` is now a thin synchronous
wrapper that builds a session and polls `isDone()`.

- **prepare:** `ydiff = diff(round(axis_cts'))` → per-sample increments; build
  `CD …;` strings; controller setup `CO 15`, `ST`, `SH ABCEFG`, optional
  `#Pulse` trigger program + `XQ`, then `CMABCEFG`, `DT n` (where
  `n = round(log2(dt·1024))`).
- **STREAMING:** poll `CM?` (free buffer slots out of 511); when ≥ `TargetBuff`
  (250), send the next chunk. Tracks `min_pending` for **buffer-health** /
  starvation warnings.
- **TAPERING (soft-stop):** when `cancel_token.requested`, scale the next
  `1/dt` increments by a half-cosine (Hann) window → velocity ramps 1×→0 over
  ~1 s, then drains.
- **DRAINING → COMPLETE:** wait for buffer to report full-empty (511 free),
  send the `CD 0,0,0,,0,0,0=0` end sentinel, clear trigger bits, `ST`,
  fire `on_complete(exitcond, record)` with `exitcond ∈ {DONE, SOFTSTOP, ERROR}`.
- Optional `record_flag` samples `TP` per chunk into `record.positions`.

### 6.5 Soft stop
`SoftStopButton` → `softStopToken.request()`. The streaming session sees the flag
on its next tick, runs the Hann taper, finishes with `exitcond = 'SOFTSTOP'`.
HexControl latches `softStopLockout`, which blocks Execute Motion until a
**Return to Datum** clears it (jogs stay enabled to walk the platform back).

### 6.6 Digital shadow (live feedback) — runs continuously at 10 Hz
```
encoderTimer tick ─▶ updateDigitalShadow(app)
   poll TP ─▶ counts (A B C E F G) ─▶ EncoderToLength ─▶ L_measured (6×1)
   write encoder + length DROs (change-detected)
   FK: ForwardKinematics_hexapod(hex_obj, L_measured, seed)   ← Newton-Raphson,
       numerical 6×6 Jacobian, warm-started from last/commanded pose, ≤8 iters
   transform platform-world pose ─▶ POI display-frame pose ─▶ write X/Y/Z/Rx/Ry/Rz
   slow tier (every 4th tick): IK on solved pose ─▶ joint-separation DROs
```
Multi-rate dispatch + per-widget change detection + skip-if-stationary keep it
cheap; FK non-convergence or a busy controller just skips the tick (DROs hold).

### 6.7 Camera pose measurement
`MeasurePoseButton` → `MeasurePoseFromCheckerboard` → `GrabCheckerboard`:
pick a PNG, `detectCheckerboardPoints` + `extrinsics` against `Basler_Params.mat`,
solve a reference rectangle's world position, convert to a datum-relative
`P_Pose`, render the image into `UIAxes3`, write `hex_obj.pose`.

### 6.8 Home & datum
Re-Home Actuators confirms retraction, sends `SH/HM/BG ABCEFG`, sets
`pose_platform = [Home_platform; 0;0;0]`, derives display pose via
`EffectorFromPlatformPose`, re-IK + replot. Set/Reset Datum buttons rewrite
`T_world_datum_platform` (see §5).

---

## 7. Galil command interface

`gclib` surface used: `GOpen`, `GClose`, `GCommand`, `GProgramDownload`, `GInfo`,
`GMotionComplete`. Responses are structs with a `.string` field
(`str2num(resp.string)`).

DMC commands exercised (see `Docs/DMC40X0-Command Reference.pdf`):

| Command | Use |
|---|---|
| `SH` / `MO` / `ST` / `AB` | servo on (unlock brakes) / off (lock) / stop / abort |
| `HM` + `BG` | home + begin motion |
| `CM ABCEFG` / `CM?` | enter contour mode / query free buffer slots (511 = empty) |
| `DT n` | contour sample period = `2^n / 1024` s |
| `CD a,b,c,,d,e,f` | contour data increments (per axis; `,,` skips D) |
| `CD 0,0,0,,0,0,0=0` | end-of-contour sentinel |
| `TP` / `TPx` | tell position (encoder counts) — feeds the digital shadow |
| `CO`, `SB`/`CB` | configure outputs / set & clear digital bits (camera trigger) |
| `XQ #Pulse,thread`, `WT`, `JP`, `EN`, `HX` | trigger-pulse program execution |

**Trigger pulse:** `#Pulse` (downloaded inline, or the standalone `PulseOut.dmc`)
toggles GPIO bits in a `JP`-loop with `WT` waits to emit a camera-sync pulse
train; cleared by `CB` + `ST` at end of stream.

---

## 8. The emulator (`GalilEmulator`)

A `handle` class implementing the same gclib surface so the *entire* GUI runs
without hardware. It keeps virtual state — `positions(1×8)` encoder counts, servo
flags, contour FIFO, GPIO bits, downloaded programs — and a **lazy virtual clock**:
on each `CM?`/command it advances simulated time and drains contour samples at
`1024/2^n` Hz, integrating their increments into `positions`. So `CM?` reports a
realistically draining buffer and `TP` returns advancing counts, which is exactly
what the streaming loop and digital shadow consume. Unknown commands are logged
and return empty (matching tolerant real-controller behavior). `galilFactory`
chooses emulator vs. real `py.gclib.py` from the Emulate checkbox.

Tested by `Emulator/tests/test_GalilEmulator.m` and
`Emulator/tests/test_stream_integration.m`.

---

## 9. Trajectory generators

- **`squashfunction`** — scaled `tanh` S-curve between two endpoints (zero
  velocity at both ends); the primitive under jog and home paths.
- **`JogFromCurrentPosition`** — 2 s relative S-curve for a single 6-vector jog.
- **`ReturnToDatumPath`** — 2 s relative S-curve back to the datum origin.
- **`Grab_Point_to_PointData`** (in app) — multi-segment table → trapezoidal
  (linear) or smoothed (squash) interpolation per segment.
- **`Grab_SineSeriesData`** (in app) — sum of sinusoids per DOF with a 2 s
  ramp-in/out envelope.
- **`Hexapod_Generate_MultiSine_Excitation.mlapp`** (standalone) → multisine
  `.mat`, converted to a `hex_path` by **`Convert_multisine_to_hex_path`**
  (currently hard-codes `dt = 1/256`).

---

## 10. External dependencies & data assets

- **gclib (Galil) Python library** — invoked through MATLAB's `py.` bridge;
  startup adds `C:\Program Files (x86)\Galil\gclib\source\wrappers\python\` to the
  path. Vendored SDK + DLLs live under `Auxiliary/gclib/` (incl. `dll/x64/*.dll`).
- **MATLAB toolboxes** — App Designer (UI), and Computer Vision / Image
  Acquisition for the camera path (`detectCheckerboardPoints`, `extrinsics`,
  `cameraParams`).
- **`pagemtimes`** (R2020b+) in the vectorized kinematics; implicit expansion
  (R2016b+) throughout.
- **Data files:** `JointSep_Interpolants.mat` (U-joint angle → yoke separation
  interpolants, required at startup), `Basler_Params.mat` (camera calibration),
  `angle_limit_UjointAB.mat`/`angle_limit_UjointCD.mat`, `TestImages_homing_camera/`
  (calibration/test PNGs), `TestMotion/` (sample profiles).
- **Offline collision study** — `CollisionMeshes/` (CAD + STL) with
  `CollisionMeshes/Code/checkYokeCollision.m` etc.; not wired into the live GUI
  (the runtime uses the interpolant-based separation check instead).

---

## 11. Module dependency map (control + feedback)

```
HexControl.mlapp
├─ InitializeHexapodObject ─────────────── hex_obj, hex_setup
├─ galilFactory ──┬─ py.gclib.py            (real)
│                 └─ GalilEmulator          (emulated)
├─ planning ──────┬─ JogFromCurrentPosition ─┐
│                 ├─ ReturnToDatumPath        ├─ squashfunction
│                 ├─ Grab_Point_to_PointData ─┘
│                 └─ Grab_SineSeriesData
├─ SimulateMotionProfile_kinematics ─┬─ composeTransform/poseToTransform/invert
│                                     ├─ E2R / R2E
│                                     └─ LengthToEncoder
├─ SimulateMotionProfile_dynamics
├─ Plot_Check_Motion ─ FindEdges
├─ StreamContourData_to_Galil ─ ContourStreamSession ─ CancelToken
├─ updateDigitalShadow ─┬─ ForwardKinematics_hexapod ─ E2R
│                        ├─ EncoderToLength
│                        └─ InverseKinematics_hexapod (slow tier)
├─ updateGauges
├─ hexapodGraphic_nocalc_replot
├─ AnimateHexapod
├─ EffectorFromPlatformPose ─ (frame helpers)
├─ MeasurePoseFromCheckerboard ─ GrabCheckerboard ─ (Basler_Params.mat)
└─ child apps: Point_to_Point, SineSeries  (call back via Grab_* )
```

---

## 12. Notable invariants & gotchas

- **Axis D is unused.** Six actuators map to Galil A B C E F G; the `,,` in `CD`
  strings and `ACTIVE_SLOTS=[1 2 3 5 6 7]` preserve the skip — string parsing
  must keep empty comma slots (`CollapseDelimiters=false` in the emulator).
- **`pose_t` is relative.** `SimulateMotionProfile_kinematics` adds `hex_obj.pose`
  to every sample; planners therefore emit *relative* trajectories (see the
  `ReturnToDatumPath` header note about avoiding a double-add).
- **Commanded vs. measured.** `updateGauges` shows the *commanded* pose; the
  digital-shadow timer shows the *measured* pose solved from encoders. They are
  separate paths writing the same DRO widgets at different times.
- **Soft-stop lockout** is a one-way latch cleared only by Return-to-Datum.
- **`.mlapp` files are binary** App Designer bundles (source lives in
  `matlab/document.xml` inside the zip); UI wiring changes must be done in App
  Designer — see `APP_DESIGNER_WIRING.md`.
```
