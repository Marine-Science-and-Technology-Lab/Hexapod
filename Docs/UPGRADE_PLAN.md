# MaST Hexapod GUI — Upgrade Plan

Post-merge plan for optimization, bugfixes, and added features. The merge consolidating Variant1 and Variant2 into [Merged/](..) is complete; this document is now the source of truth for follow-up work. All file paths are relative to the repo root unless noted.

## Context

[Merged/](..) is the canonical tree going forward. During exploration for the merge we identified:

- Five regressions V2 introduced that the merge already restores (emulation mode, LaTeX labels, Galil trigger-pulse arg, Save/Load point-sequence buttons, button color styling). Those are fixed.
- A set of residual performance, correctness, and architecture issues that pre-date the split. Those are captured here, ordered roughly by effort/value.
- A fiducial-tracking code path (`GrabCheckerboard`, `FindEdges`, `Camera_Recording`) that is *implemented* but not *wired up* to any UI control.

On top of that, four **net-new capabilities** are scoped in **[Phase 5](#phase-5--major-new-features)**:
1. **Galil emulator** — a software stand-in for the physical controller, same API, usable for dry-testing.
2. **Animate Motion** — popup 3D playback window with scrubbable time bar and transport controls.
3. **Forward kinematics + digital shadow** — iterative solver running in real time alongside motion streaming.
4. **Soft-stop** — graceful 1-second tapered ramp-down instead of hard `ST`.

Galil motion controller reference for the `gclib`/DMC commands used in the streaming code: [Merged/Docs/DMC40X0-Command Reference.pdf](DMC40X0-Command%20Reference.pdf). Cite it by command (e.g. `CM`, `CD`, `DT`, `SB`, `CB`, `WT`, `XQ`) when touching [StreamContourData_to_Galil.m](../StreamContourData_to_Galil.m) or the `#Pulse` program.

---

## Phase 1 — Bugfixes & quick wins

Low effort, no architectural change. These should land first.

### 1.1 Hard-coded user path in Camera_Recording.m
- **Where:** [Merged/Camera_Recording.m:10](../Camera_Recording.m#L10) — path is `C:\Users\juruiz\Desktop\TestBasler`.
- **Fix:** accept an output-folder argument (default to a `Recordings/` sibling of the script, or call `uigetdir` if launched interactively).
- **Acceptance:** running `Camera_Recording` on a fresh machine must not require editing source.

### 1.2 Unused `record_flag` parameter
- **Where:** [Merged/StreamContourData_to_Galil.m:1](../StreamContourData_to_Galil.m#L1) — function signature declares `record_flag` but the body never references it.
- **Fix:** wire it to `RA`/`RD` record-array commands (see the **Data Record** / **Record Array** sections of [the Galil command reference](DMC40X0-Command%20Reference.pdf)) so the controller streams position feedback back to the host.
- **Why kept, not dropped:** this is the upstream data feed for the **digital-shadow forward-kinematics loop (§5.3)**. Dropping the param would re-open that feature later.
- **Acceptance:** `record_flag=1` produces an in-memory array of actual encoder positions aligned to the commanded contour; sample cadence matches `DT`.

### 1.3 Dead comment waitbar lines
- **Where:** [Merged/StreamContourData_to_Galil.m:38,43,62-64,72-75](../StreamContourData_to_Galil.m#L38) and [Merged/CollisionMeshes/Code/Animate_Collision.m:38](../CollisionMeshes/Code/Animate_Collision.m#L38).
- **Fix:** delete the commented-out `waitbar`/`ishandle` blocks or re-enable them behind a `verbose` flag. Clutter only — no functional impact.

### 1.4 GrabCheckerboard dead outputs
- **Where:** [Merged/GrabCheckerboard.m:70-73](../GrabCheckerboard.m#L70-L73) — `CamPose` and `Trans_camplatform` are computed but `P_Pose` ignores them.
- **Fix:** decide whether those intermediate poses contribute to the returned platform pose. If yes, combine them (likely `P_Pose = Trans_camplatform * CamPose` or similar homogeneous composition); if no, delete.
- **Acceptance:** call `GrabCheckerboard` on a known-pose image from [Merged/TestImages_homing_camera/](../TestImages_homing_camera/) and compare against the ground-truth pose from the calibration metadata.

### 1.5 Wire GrabCheckerboard into the Camera tab
- **Where:** [Merged/HexControl.mlapp](../HexControl.mlapp) — the `CameraTab` has axes and two legacy buttons but no measure-pose control. Line ~348 of the App Designer source has a commented `[P_Pose camimg]=GrabCheckerboard(app.hex_obj);` call.
- **Fix:** add a **Measure Pose** button that:
  1. Calls `GrabCheckerboard(app.hex_obj)`.
  2. Renders `camimg` into `app.UIAxes3`.
  3. Updates `app.hex_obj.pose` and refreshes the pose readout.
  4. Displays a warning if corner detection fails (returns empty).
- **Acceptance:** press the button with a checkerboard in view → live image shown, pose fields populate, status LED or text confirms success.

### 1.6 Batch STL discovery
- **Where:** [Merged/CollisionMeshes/Code/Collision_LoadMeshes_Hexapod.m:1-4](../CollisionMeshes/Code/Collision_LoadMeshes_Hexapod.m#L1-L4) — four sequential `uigetfile` dialogs.
- **Fix:** auto-discover via `dir(fullfile(meshRoot,'Yoke*_*.STL'))` and group by prefix (A/B/C/D). Fallback to `uigetdir` if no match.
- **Acceptance:** `Collision_LoadMeshes_Hexapod` runs without user prompts against the shipped [Merged/CollisionMeshes/](../CollisionMeshes/) tree.

---

## Phase 2 — Performance

All items target the motion-profile simulation path — the common bottleneck when the user asks for a long trajectory preview.

### 2.1 Vectorize the kinematics time-loop
- **Where:** [Merged/SimulateMotionProfile_kinematics.m:48-93](../SimulateMotionProfile_kinematics.m#L48-L93) — outer `for j = 1:size(r,2)` over every timestep wrapping an inner `for i = 1:6` over joints. Per sample: one `E2R` call, one `plat_CM` computation, six platform-link rotations, six length calculations, four `acos` U-joint angles.
- **Fix:** vectorize in two passes.
  1. Batch `E2R` across all samples (build a 3×3×N rotation tensor); use `pagemtimes` (R2020b+) for `R*plat`.
  2. Vectorize the six-joint inner loop as a single `3×6×N` tensor multiply plus `vecnorm` for link lengths and elementwise `acos` for U-joint angles.
- **Acceptance:** 10k-sample trajectory completes in under 10 % of the current wall time; `isequal(result_new, result_old)` within `1e-10` tolerance against a canonical motion from [Merged/TestMotion/](../TestMotion/).

### 2.2 repmat → implicit expansion
- **Where:**
  - [Merged/SimulateMotionProfile_dynamics.m:19](../SimulateMotionProfile_dynamics.m#L19) — `repmat([0;0;9.81],1,length(r_ddt))`.
  - [Merged/InitializeHexapodObject.m:52-54](../InitializeHexapodObject.m#L52-L54) — two `repmat([0;0;Zlink],1,6)` calls.
  - Any other `repmat(v, 1, N)` where `v` is a column vector being added or multiplied with an `M×N` matrix.
- **Fix:** drop `repmat`; rely on MATLAB R2016b+ implicit broadcasting (`M + v`).
- **Acceptance:** no `repmat(.*,1,.*)` for broadcast purposes in the simulation path; `isequal` against prior outputs.

### 2.3 Graphics pipeline consolidation
- **Where:** [Merged/hexapodGraphic.m](../hexapodGraphic.m) vs [Merged/hexapodGraphic_nocalc_replot.m](../hexapodGraphic_nocalc_replot.m).
- Note: the inferior `hexapodGraphic_nocalc.m` was dropped during the merge — already done.
- **Fix:** decide canonical. `_replot` is faster for streaming; `hexapodGraphic` is stateless and safer for one-off renders. Suggested: keep `_replot` as the canonical real-time updater; add a thin `hexapodGraphic_init` that creates the objects once and returns handles; deprecate `hexapodGraphic.m` (or restrict it to an `if nargin<handles` fallback branch).
- **Acceptance:** one documented entry point; callers updated; single code path for the animation loop.

### 2.4 Convert_multisine_to_hex_path sampling validation
- **Where:** [Merged/Convert_multisine_to_hex_path.m:9-32](../Convert_multisine_to_hex_path.m#L9-L32) — hard-coded `dt = 1/256` with no validation against the input.
- **Fix:** infer `dt` from the multisine struct's time vector (or a new `Fs` field emitted by `Hexapod_Generate_MultiSine_Excitation.mlapp`); error out loudly if 256 Hz is assumed but the source data disagrees.
- **Acceptance:** feeding a 1 kHz multisine through the converter produces a `hex_path` with `dt = 1e-3` rather than silently resampling to 256 Hz.

---

## Phase 3 — Correctness & new features

Each item adds a small capability and/or closes a correctness gap.

### 3.1 Interpolant-vs-mesh collision regression test
- **What:** the runtime joint-separation check at [Merged/Plot_Check_Motion.m:3](../Plot_Check_Motion.m#L3) uses interpolant lookups (`JointSep_Interpolants.mat`), while the STL-based `checkYokeCollision` in [Merged/CollisionMeshes/Code/](../CollisionMeshes/Code/) is only invoked from standalone demo scripts. The two pipelines are never cross-validated.
- **Fix:** add `tests/test_collision_agreement.m` that:
  1. Loads every motion profile in [Merged/TestMotion/](../TestMotion/).
  2. Runs both the interpolant check (`Plot_Check_Motion`-style) and the geometric check (`checkYokeCollision`) at every timestep.
  3. Asserts the boolean verdicts agree and the distances correlate above a threshold (e.g. `r > 0.95`).
- **Acceptance:** test passes on every shipped profile; a disagreement prints the timestep + pose + separation distance.

### 3.2 Migrate TestCheckerboardRealTime.m off legacy ImaqScript
- **Where:** [Merged/TestCheckerboardRealTime.m:8,37](../TestCheckerboardRealTime.m#L8) — `ImaqScript` is the deprecated Image Acquisition Toolbox scripting layer.
- **Fix:** use `videoinput`/`imaqhwinfo` with the Basler adapter (or the GenTL API Basler ships in pylon). Keep the 30 fps loop target; preserve the existing `GrabCheckerboard` call per frame.
- **Acceptance:** script runs against live Basler hardware without Deprecation warnings; unchanged frame-rate and corner-detection output.

### 3.3 Integrate camera calibration workflow
- **Where:** [Merged/TestImages_homing_camera/](../TestImages_homing_camera/) — 40 BaslerCalibration images; [Merged/Basler_Params.mat](../Basler_Params.mat) is the current calibration.
- **Fix:** add `tools/recalibrate_camera.m` that runs `cameraCalibrator` programmatically on the shipped images and writes a fresh `Basler_Params.mat`, with reprojection-error reporting.
- **Acceptance:** recalibration produces params within ~1 px reprojection error; on-disk `.mat` schema compatible with `GrabCheckerboard`.

### 3.4 Galil command-string validation
- **Where:** [Merged/StreamContourData_to_Galil.m:18-20](../StreamContourData_to_Galil.m#L18-L20) builds the `CD` (contour data) command as string concatenation: `CD a,b,c,,d,e,f`. The double-comma skips axis D (correct for this hexapod's ABCEFG axis layout), but this is magical and fragile — a change in axis wiring silently produces wrong motion.
- **Fix:** encode the active axis mask explicitly (e.g. `axes = 'ABCEFG'`) and derive the `CD` positional template from it. Cross-reference the `CM` (Contour Mode) call on [line 31](../StreamContourData_to_Galil.m#L31) which uses the same axis set, and keep them in sync from one source of truth. Cite `CD` and `CM` semantics in [Merged/Docs/DMC40X0-Command Reference.pdf](DMC40X0-Command%20Reference.pdf).
- **Acceptance:** unit test builds the CD string for axes `ABCEFG` and `ABCDEF` and asserts the positional placeholders match.

### 3.5 `#Pulse` trigger program robustness
- **Where:** [Merged/StreamContourData_to_Galil.m:26](../StreamContourData_to_Galil.m#L26) downloads a DMC program that pulses GPIO bits 17, 25, 33 with `WT16,1` inter-pulse waits. The program runs forever until `ST` (line 81) plus `CB 17/25/33` clear it.
- **Fix:** confirm the `WT16,1` timing against the desired camera frame rate (see `WT` and `TM` in [the reference PDF](DMC40X0-Command Reference.pdf)). Consider replacing the free-running loop with an `MG{Z10.0}` or `WT` tied to contour index so pulses are phase-locked to motion samples. Also ensure `EN` cleanly exits if `ST` fires mid-pulse.
- **Acceptance:** scope trace shows the pulse train matches the contour sample rate; stopping motion leaves all three bits low.

### 3.6 Pose readout precision
- **Where:** the LaTeX-labelled edit fields for mass / inertia / force / moment in the HexControl `MassDistributionTab` and `ExternalForceTab`. No action yet — just capture that label rendering is now correct; double-check unit conversions in [Merged/InitializeHexapodObject.m](../InitializeHexapodObject.m) if any dynamics feature depends on SI.

---

## Phase 4 — Architecture

Bigger decisions with cross-cutting impact. Pick up after Phases 1–2 are green.

### 4.1 Resolve SineSeries vs standalone multisine app
- **What:** [Merged/SineSeries.mlapp](../SineSeries.mlapp) (inside HexControl flow) and [Merged/Hexapod_Generate_MultiSine_Excitation.mlapp](../Hexapod_Generate_MultiSine_Excitation.mlapp) (standalone) both generate sinusoidal / multisine excitations.
- **Decide:** one of
  1. Deprecate `SineSeries` inside HexControl — external app is the canonical tool; HexControl only consumes its output via [Merged/Convert_multisine_to_hex_path.m](../Convert_multisine_to_hex_path.m).
  2. Keep both but document the division of labor (e.g. `SineSeries` = single-axis swept tone, multisine app = multi-channel broadband excitation).
- **Acceptance:** README clearly describes which tool to use for which workflow; deprecated UI controls removed or clearly labeled.

### 4.2 Unify camera workflow
- **What:** fiducial tracking code lives in standalone `.m` files ([Camera_Recording.m](../Camera_Recording.m), [GrabCheckerboard.m](../GrabCheckerboard.m), [FindEdges.m](../FindEdges.m), [TestCheckerboardRealTime.m](../TestCheckerboardRealTime.m)) but the HexControl `CameraTab` only wires a few of them.
- **Decide:** either
  1. Fold all camera features into the HexControl `CameraTab` (calibration, live preview, homing measurement, recording).
  2. Spin off a separate `HexCamera.mlapp` analogous to the multisine generator.
- **Acceptance:** one canonical place for the camera GUI; the standalone `.m` scripts become library functions, not duplicate entry points.

### 4.3 Emulation-mode coverage (superseded by §5.1)
- **What:** V1 restored the `if ~app.Emulated ... else "Emulation mode"` branch around controller init ([Merged/HexControl.mlapp](../HexControl.mlapp) initialize-controller callback). Several other callbacks currently assume a live Galil connection and will throw in emulation.
- **New approach:** instead of stubbing each call site with `if app.Emulated; return; end`, build a proper **Galil emulator (§5.1)** that implements the same `gclib` surface. Then `app.g` can simply be bound to the emulator in emulated mode and every callback "just works" unchanged.
- **Residual work after §5.1 lands:** sweep the callback table and remove legacy `if app.Emulated` stubs that are now dead weight; keep only a single mode switch at controller-init time.
- **Acceptance:** full GUI workflow runs end-to-end against the emulator with zero `if app.Emulated` branches outside `InitializeControllerButtonPushed`.

### 4.4 gclib dependency manageability
- **What:** [Merged/Auxiliary/gclib/](../Auxiliary/gclib/) is a vendored Galil SDK tree (~tens of MB). Current approach: commit as-is.
- **Revisit:** if/when we start versioning more actively, consider pinning a specific gclib release via a pointer file + release link, or moving the tree to Git LFS. Not urgent.

---

## Phase 5 — Major new features

Four user-requested capabilities that extend the GUI beyond its current scope. Each is big enough to warrant its own milestone.

### 5.1 Galil emulator module
**Goal:** dry-test the entire GUI without hardware by replacing the physical controller with a software emulator that accepts the same `gclib` API and returns plausible responses.

**Design:**
- New class `Merged/Emulator/GalilEmulator.m` implementing the subset of the `gclib` / `py.gclib.py` interface that HexControl actually calls: `GCommand`, `GProgramDownload`, `GInfo`, `GMotionComplete`, `GOpen`, `GClose`.
- Internal state:
  - `positions(6)` — simulated encoder counts per axis (ABCEFG mapped to indices 1–6, skipping D).
  - `contour_buffer` — FIFO of pending `CD` commands, drained against a virtual clock at rate `1 / (DT × sample_time_base)`.
  - `gpio(8)` — 0/1 bits updated by `SB`/`CB`.
  - `programs` — map of label → DMC text loaded via `GProgramDownload`.
  - `mode` — `'idle' | 'contour' | 'position' | 'jog'`.
  - `clock` — wall-clock reference for buffer draining; `tic`/`toc` based so it advances only when the caller asks for status.
- Command handlers — one MATLAB function per command family. Minimum set to cover current callers (cross-check against [DMC40X0-Command Reference.pdf](DMC40X0-Command%20Reference.pdf) for exact semantics):
  - Servo: `ST`, `SH`, `MO`, `AB`.
  - Contour: `CM`, `CMABCEFG`, `CM?`, `CD <pts>`, `CD 0,0,0,,0,0,0=0`, `DT <n>`.
  - Position: `PA`, `PR`, `BG`, `TP`, `TPA`..`TPG`.
  - I/O: `SB <n>`, `CB <n>`, `CO <mask>`, `MG @IN[<n>]`.
  - Program: `XQ #<label>,<thread>`, `HX`, `EN`, `JP`, `WT`, `AM`.
  - Unknown commands → log to a ring buffer and return empty (matches the tolerant behavior of real `gclib`).
- Virtual physics: drain one `CD` sample from the buffer every `DT` virtual-seconds; update `positions` by the accumulated delta. `CM?` returns `512 - buffered_samples`.
- Optional fault injection: properties like `EmuInjectBufferStarve`, `EmuInjectAxisFault` for regression testing of error paths.

**Integration:**
- Add a `Merged/Emulator/galilFactory.m` that returns either `py.gclib.py()` or `GalilEmulator()` based on `app.Emulated`.
- [Merged/HexControl.mlapp](../HexControl.mlapp) `InitializeControllerButtonPushed` callback calls the factory; downstream code is unchanged.
- All callbacks that currently do `app.g.GCommand(...)` need no modification.

**Acceptance:**
- Launch HexControl in emulation, initialize, generate a multi-segment motion, execute it — see [§5.2 Animate Motion](#52-animate-motion-popup-3d-render) play through the trajectory and the emulator's internal `positions` match the commanded endpoint within ±1 count.
- `Merged/Emulator/tests/` passes: individual tests for `CM?` buffer draining, `ST` mid-contour, `SB`/`CB` GPIO round-trip, `#Pulse` program download + `XQ` execution.
- Grep confirms no `if app.Emulated; ... end` branches remain outside `InitializeControllerButtonPushed` (closes §4.3).

**Dependencies:** none upstream. Upstream of §5.2, §5.3, §5.4 testing.

---

### 5.2 Animate Motion — popup 3D render
**Goal:** a separate figure window that plays the generated motion profile as a 3D hexapod animation, with scrubbable time bar and transport controls.

**Design:**
- New App Designer app `Merged/AnimateMotion.mlapp` (kept separate from HexControl for clean lifecycle management — can open multiple animation windows, close independently).
- Layout:
  - 3D axes (majority of window).
  - Bottom bar: **◀◀** (jump start) — **▶/⏸** (play/pause) — **▶▶** (jump end) — slider (0 → N_samples-1) — time readout (`hh:mm:ss.mmm / total`) — speed dropdown (0.25×, 0.5×, 1×, 2×, 4×, 10×) — loop toggle.
- Graphics:
  - Use `hexapodGraphic_nocalc_replot.m` for in-place `.XData`/`.YData` updates (fast).
  - On open, pre-compute every pose's link geometry once (vectorized — see §2.1) and cache as a struct array indexed by frame.
  - Initial render via a new `hexapodGraphic_init` helper (part of §2.3 consolidation) that creates all `line`/`patch`/`surf` handles and returns a handle struct.
- Playback:
  - `timer` object at 30 Hz real-time; each tick advances the frame index by `round(speed × playback_dt / animation_dt)` samples.
  - Slider drag → pause + jump to frame; slider release resumes if was playing.
  - Play/pause toggles the timer's `Running` state.
- Opened from a new **Animate Motion** button on the HexControl `PlanMotionTab`, next to Execute/Export. Enabled only when `app.hex_path` is populated.

**Files to add:**
- [Merged/AnimateMotion.mlapp](../AnimateMotion.mlapp)
- `Merged/hexapodGraphic_init.m` (helper created during §2.3)

**Dependencies:**
- §2.3 graphics consolidation (clean entry point). Can be worked around initially by calling `hexapodGraphic_nocalc_replot.m` directly with a one-shot creation branch.
- §2.1 vectorized kinematics (nice-to-have for smooth scrubbing of 100k-sample paths; not a hard blocker for shorter test profiles).

**Acceptance:**
- Open HexControl in emulation (§5.1), load any [Merged/TestMotion/](../TestMotion/) profile, press Animate → popup opens, hexapod animates at real-time rate.
- Drag slider — render tracks immediately, no lag for ≤10k samples.
- Pause, seek, resume — playback continues from the scrub point.
- Close animate-window while playing — no errors; timer cleaned up.

---

### 5.3 Forward kinematics + real-time digital shadow
**Goal:** a fast iterative forward-kinematics solver that, given the 6 actuator encoder readings and a good initial pose guess, returns the platform pose. Run it continuously alongside motion streaming so the GUI displays where the hexapod *actually is*, not just where it was commanded to go.

**Algorithm:**
- Residual: `f(X) = IK(X) - L_measured`, where `IK` is existing [Merged/InverseKinematics_hexapod.m](../InverseKinematics_hexapod.m) (returns 6 link lengths for pose `X = [x,y,z,roll,pitch,yaw]`), and `L_measured = LengthFromEncoder(counts)` (inverse of existing [Merged/LengthToEncoder.m](../LengthToEncoder.m)).
- **Newton-Raphson** with numerical Jacobian (6×6):
  - `J(i,j) ≈ (IK(X + e_j δ) - IK(X - e_j δ)) / (2δ)`, `δ = 1e-5` per DOF.
  - Update: `X ← X - J \ f(X)`; iterate until `‖f‖∞ < tol` (`tol = 1e-6 m`).
  - Seeded from `X_prev` (last known pose) → typically converges in 1–2 iterations during smooth motion.
- **Fast path:** if `‖L_measured - L_prev‖ < ε`, skip solve and reuse `X_prev`.
- **Safety:** cap iterations (e.g. 8); on non-convergence, fall back to the commanded pose and log a warning (don't crash the streaming loop).

**Files to add:**
- [Merged/ForwardKinematics_hexapod.m](../ForwardKinematics_hexapod.m) — solver function `[X, converged, iters] = ForwardKinematics_hexapod(hex_obj, L_measured, X_seed, opts)`.
- [Merged/DigitalShadow.m](../DigitalShadow.m) — wrapper that owns a `timer`, pulls encoder counts via the emulator / real controller's `TP` command, runs the solver, and writes the result to `app.hex_obj.pose_shadow`.
- Unit tests: for a set of poses from [Merged/TestMotion/](../TestMotion/), round-trip `X → IK → FK → X'` and assert `‖X' - X‖ < 1e-8`.

**UI integration:**
- New read-only pose readout group on HexControl (or a new `DigitalShadowTab`): current commanded pose vs shadow pose, and their difference per DOF.
- Indicator LED for "shadow converged this tick" / "shadow lagged."
- Optional overlay in §5.2's Animate Motion window: ghost platform at shadow pose drawn translucent over the commanded platform.

**Target solver cadence:** match the contour sample rate (256 Hz) if possible. A rough budget for the full loop (encoder read + solve + UI update): 1–2 ms. The numerical Jacobian costs 12 IK evaluations; analytic Jacobian is a follow-up optimization if numerical proves too slow.

**Dependencies:**
- §1.2 `record_flag` must be wired first so there's actually an encoder-stream source — or, in emulation, §5.1's emulator must expose `TP` with advancing positions.
- §2.1 vectorized IK helps — Jacobian evaluation at a single pose is a 6-column batch, trivially vectorizable.

**Acceptance:**
- Unit-test round-trip passes across all TestMotion profiles.
- During emulated execution of `Circle_Test.mat`, digital-shadow pose tracks commanded pose with < 1 µm position error and < 1e-6 rad orientation error (emulator is deterministic; this validates solver correctness).
- Solver runs ≥100 Hz sustained without dropping encoder samples.

---

### 5.4 Soft-stop button
**Goal:** user presses **Soft Stop** during a running contour; the streamed motion ramps to a halt over 1 second instead of stopping instantly.

**Design:**
- Current [Merged/StreamContourData_to_Galil.m:41-65](../StreamContourData_to_Galil.m#L41-L65) is a blocking `while` loop that streams `TargetBuff`-sized chunks of `CD` commands until the buffer catches up to the full `posStr` array.
- Add a cancel token: an app-level property `app.soft_stop_requested` (or a shared handle object passed into the function so it can be toggled from the main UI thread).
- Restructure the streaming loop so that each iteration:
  1. Checks `soft_stop_requested`.
  2. If true, compute the remaining portion to send as a **1-second taper** and break out of the main streaming loop into a dedicated taper-and-finish branch.
- Taper synthesis:
  - `N_taper = round(1 / hex_path.dt)` samples.
  - Take the next `N_taper` rows of `ydiff` (the *incremental* position commands — so tapering them scales velocity per sample, not absolute position).
  - Apply a **half-cosine (Hann) ramp** `w(k) = 0.5 * (1 + cos(pi * k / (N_taper-1)))` for `k = 0..N_taper-1`, so velocity smoothly transitions from 1× → 0 with zero derivative at both ends.
  - Push the tapered rows through the same `CD` streaming path.
- After taper:
  - Wait for buffer to drain (reuse lines 69-76's drain loop).
  - Issue `CD 0,0,0,,0,0,0=0` (end-of-contour sentinel) and `ST`.
  - Clear trigger bits `CB 17 / 25 / 33` and stop `#Pulse` if running.
- **GUI responsiveness:** since the current streaming loop blocks MATLAB, the Soft Stop button won't register mid-stream. Two options:
  1. **Preferred:** convert streaming to a MATLAB `timer` / periodic callback so the GUI remains responsive; button click sets the flag, which the timer sees next tick.
  2. **Minimal:** call `drawnow limitrate` each iteration of the current loop so UI callbacks can fire.
  - Start with option 2 as the quick unblocking change; migrate to option 1 as part of the same feature work if responsiveness proves inadequate.

**Files to modify:**
- [Merged/StreamContourData_to_Galil.m](../StreamContourData_to_Galil.m) — add cancel-token parameter, taper branch, `drawnow limitrate`.
- [Merged/HexControl.mlapp](../HexControl.mlapp) — add **Soft Stop** button on `PlanMotionTab`, red, enabled only while streaming; callback sets the flag.
- [Merged/StreamContourData_to_Galil_w_trigger.m](../StreamContourData_to_Galil_w_trigger.m) — propagate the same change.

**Acceptance:**
- Against the emulator (§5.1): start a 10 s trajectory, press Soft Stop at t ≈ 3 s → virtual encoder trace shows smooth deceleration over ~1 s, no jerk discontinuity, final velocity 0. Emulator logs the `ST` command arriving after the taper samples, not before.
- Against real hardware: same behavior observed on scope / DAQ.
- Edge case: if fewer than `N_taper` samples remain in `posStr`, taper the available remainder. No out-of-bounds indexing.
- Edge case: Soft Stop pressed *before* streaming actually starts — no-op (or issues only `ST` + bit clears).

**Dependencies:**
- §5.1 emulator for end-to-end testing without risking hardware.
- Phase 1.3 dead-waitbar cleanup is cosmetic but should be folded in since the streaming loop is being touched.

---

## Deferred / open questions

- What MATLAB release is the target? `pagemtimes` requires R2020b+; some implicit-expansion idioms need R2016b+. The answer drives Phase 2.1.
- Is the Galil hardware always the DMC-4040 (per the reference PDF) or do different rigs use other controllers with subtly different command sets?
- How often is camera calibration expected to be redone? That affects whether `tools/recalibrate_camera.m` should ship as a one-click app or stay a script.

---

## Suggested execution order

1. **Phase 1 (except 1.2)** — days of work, all low risk; unblocks testing.
2. **§5.1 Galil emulator** — next, because it unlocks dry-testing of every feature below without hardware.
3. **§1.2 `record_flag`** — now that the emulator can simulate a `TP`/record stream, wire up the real feedback path.
4. **§2.1 + §2.2** — highest perf impact, bounded scope. §2.1 also speeds up §5.3's Jacobian.
5. **§5.4 Soft-stop** — after §5.1 so we can validate against the emulator before risking hardware.
6. **§3.1** — regression safety net before any further changes to simulation or collision code.
7. **§2.3 graphics consolidation + §5.2 Animate Motion** — ship together; §5.2 drives the §2.3 refactor.
8. **§5.3 Forward kinematics + digital shadow** — depends on §1.2, §2.1, §5.1.
9. **§3.2, §3.3** — camera feature completeness.
10. **§2.4, §3.4, §3.5** — polish and Galil robustness.
11. **§4.1, §4.2** — architectural cleanup once the smaller items stop moving around.
12. **§4.3** — emulation-branch sweep; mostly already-dead code once §5.1 lands.
