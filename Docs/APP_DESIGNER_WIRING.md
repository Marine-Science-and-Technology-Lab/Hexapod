# App Designer Wiring Tasks

`.mlapp` files are binary App Designer bundles; they can only be edited inside MATLAB's App Designer. This doc tracks UI changes the upgrade plan has prepared backing `.m` code for, but which still need to be wired up manually.

Open an app with `appdesigner HexControl.mlapp` (or Point_to_Point.mlapp, etc.).

---

## Pending

### §1.5 — Measure Pose button on HexControl Camera tab

**Backing function (already landed):** [../MeasurePoseFromCheckerboard.m](../MeasurePoseFromCheckerboard.m)

**Steps in App Designer:**

1. Open `HexControl.mlapp`.
2. Switch to **Design View**, select the **Camera** tab.
3. Drag a **Button** onto the Camera tab; place it below the existing buttons (e.g. near the "Get Current Platform Position" / "Home Generate Path" pair).
4. In the Component Browser, rename the button to `MeasurePoseButton`.
5. Set its **Text** property to `Measure Pose`.
6. Right-click the button → **Callbacks** → **Add ButtonPushedFcn**.
7. Paste this body into the generated callback:

   ```matlab
   function MeasurePoseButtonPushed(app, event)
       [app.hex_obj, status] = MeasurePoseFromCheckerboard( ...
           app.hex_obj, app.UIAxes3);
       if status.ok
           app.ControllerResponseTextArea.Value = status.msg;
           % If a pose-readout UI group exists, push values into it here.
       else
           app.ControllerResponseTextArea.Value = status.msg;
           uialert(app.UIFigure, status.msg, 'Measure Pose', 'Icon', 'warning');
       end
   end
   ```

8. Save (`Ctrl+S`). Close App Designer.

**Smoke test:**
- Launch `HexControl` (emulation is fine).
- Switch to the **Camera** tab.
- Press **Measure Pose**; pick any PNG from `TestImages_homing_camera/`.
- The captured image should render in `UIAxes3` and the status text area should show a pose readout. If the selected image has no visible checkerboard, a warning dialog appears.

---

### §5.1 — Swap controller init to use galilFactory

**Backing function (already landed):** [../Emulator/galilFactory.m](../Emulator/galilFactory.m), wrapping [../Emulator/GalilEmulator.m](../Emulator/GalilEmulator.m).

**Current `InitializeControllerButtonPushed` (roughly what's in the app today):**

```matlab
function InitializeControllerButtonPushed(app, event)
    app.LED_stat.ControllerInitialized = 0; UpdateLEDs(app); app.Connected = 0;

    g = py.gclib.py;            % create Python gclib handle
    app.g = g;
    tf = isa(app.g, 'py.gclib.py');

    if ~app.Emulated
        if tf
            try
                g.GClose;                     % clear any lingering open
                g.GOpen('192.168.42.2');      % connect to hardware
                ss = g.GInfo;
                app.ControllerResponseTextArea.Value = ss.string;
                app.LED_stat.ControllerInitialized = 1;
                UpdateLEDs(app);
                app.Connected = 1;
            catch
                app.ControllerResponseTextArea.Value = 'Controller not responding';
                app.LED_stat.ControllerInitialized = 0.6;
                UpdateLEDs(app);
            end
        else
            app.ControllerResponseTextArea.Value = 'Unable to load controller Python library';
        end
    else
        app.ControllerResponseTextArea.Value = 'Emulation mode';
        app.LED_stat.ControllerInitialized = 1;
        UpdateLEDs(app);
        app.Connected = 1;
    end
end
```

**Answers to the two questions you raised:**

1. **What replaces `g.GClose` / `g.GOpen`?** Nothing explicit — the factory handles both. `galilFactory(app.Emulated, ip)` constructs a *fresh* handle (either `GalilEmulator()` or `py.gclib.py()`) and calls `.GOpen(ip)` on it internally. The old `GClose` immediately before `GOpen` was a defensive reset on the *existing* Python handle; because the factory builds a new handle from scratch, there's nothing to pre-close. If you want to be safe against re-pressing **Initialize Controller** while a connection is live, explicitly close the previous `app.g` *before* calling the factory — see the rewrite below.

2. **Where does `ControllerIPEditField` live?** It doesn't — I invented it. The app today hardcodes `'192.168.42.2'` at [/tmp/hex2/HexControl.m:370 equivalent — inside the `try` block]. Keep it hardcoded for now; adding a UI field for the IP is a separate future task. If/when you do add one, name it `ControllerIPEditField` and the call site becomes `galilFactory(app.Emulated, app.ControllerIPEditField.Value)`.

**New `InitializeControllerButtonPushed` — paste this verbatim:**

```matlab
function InitializeControllerButtonPushed(app, event)
    app.LED_stat.ControllerInitialized = 0; UpdateLEDs(app); app.Connected = 0;

    % Defensive: close any previously-open handle. This no-ops safely
    % whether app.g is still the initial 0, a dead py.gclib.py, or a
    % live emulator.
    try; app.g.GClose; catch; end %#ok<NOSEMI>

    try
        g = galilFactory(app.Emulated, '192.168.42.2');
        app.g = g;
        ss = g.GInfo;
        app.ControllerResponseTextArea.Value = ss.string;
        app.LED_stat.ControllerInitialized = 1;
        UpdateLEDs(app);
        app.Connected = 1;
    catch ME
        if app.Emulated
            app.ControllerResponseTextArea.Value = ...
                sprintf('Emulator failed to start: %s', ME.message);
        else
            app.ControllerResponseTextArea.Value = 'Controller not responding';
        end
        app.LED_stat.ControllerInitialized = 0.6;
        UpdateLEDs(app);
    end
end
```

**What this does:**
- The first `try/catch` closes any previously-held handle — emulator or real — so tapping **Initialize Controller** twice in a row is safe.
- `galilFactory(app.Emulated, '192.168.42.2')` constructs the right kind of handle and opens it. On the real path it returns a `py.gclib.py` opened at the hardware IP; on the emulated path it returns a `GalilEmulator` opened at the sentinel address `'emulator'`.
- `g.GInfo` works identically on both: real controller returns the Galil identity string; emulator returns `'GalilEmulator, rev 0, <address>, hostname'`.
- The LED/LedStat/Connected bookkeeping is unchanged.

**Notes:**
- You no longer need the `isa(app.g, 'py.gclib.py')` check — that test guarded against a failed Python-library load. On the emulated path the factory never touches Python, and on the real path the factory still calls `py.gclib.py` and will throw into the `catch` if Python isn't configured, giving the same user-visible outcome ("Controller not responding").
- Every other callback in the app still uses `app.g.GCommand(...)` / `GProgramDownload` / `GMotionComplete` unchanged — those now transparently route to whichever implementation the factory returned.
- The old `if ~app.Emulated ... else "Emulation mode" end` branch is gone because the emulator itself *is* the "emulation mode" — its `GInfo` string and state now provide all the feedback the old hardcoded message used to.

**Smoke test (no hardware needed):**
- Launch `HexControl`, tick Emulation, press **Initialize Controller** — the response text area should show `GalilEmulator, rev 0, ...` and `ControllerInitialized = 1`.
- Press it a second time — no error, same result (exercises the defensive `GClose`).
- Generate or load any path, press **Execute Motion** — the streaming loop should run to completion. `CM?` responses drain the buffer over wall time; `TP` returns advancing encoder counts.

**Unit tests:** run [../Emulator/tests/test_GalilEmulator.m](../Emulator/tests/test_GalilEmulator.m) from MATLAB; 11 tests should pass.

---

### §5.4 — Soft Stop button on HexControl PlanMotion tab

**Backing code (already landed):**
- [../CancelToken.m](../CancelToken.m) — the shared flag object.
- [../StreamContourData_to_Galil.m](../StreamContourData_to_Galil.m) — now accepts a `cancel_token` 5th argument and performs a Hann-tapered ramp-down when it's tripped.

**Steps in App Designer:**

1. Open `HexControl.mlapp` → Design View → **Plan Motion** tab.
2. Drag a **Button** onto the tab near **Execute Motion** (position is your preference; Execute Motion is lower on the tab, Export motion profile is higher — put Soft Stop somewhere it won't get misclicked).
3. Rename it `SoftStopButton`; set **Text** to `Soft Stop`, **FontWeight** to bold, **BackgroundColor** to red (`[1 0.3 0.3]`).
4. Add a new property in Properties (Access = public):
   ```matlab
   softStopToken   % CancelToken
   ```
5. In `StartupFcn` (or the controller-init callback), initialize it once:
   ```matlab
   app.softStopToken = CancelToken();
   ```
6. Update the two `StreamContourData_to_Galil` call sites in the app so they pass the token.

   **Call site A** — inside `ExecuteMotionButtonPushed` (around line 582 of the extracted source), currently:
   ```matlab
   StreamContourData_to_Galil(g, app.hex_path, 1);
   ```
   Replace with:
   ```matlab
   app.softStopToken.reset();
   [exitcond, ~] = StreamContourData_to_Galil( ...
       g, app.hex_path, 1, 0, app.softStopToken);
   if strcmp(exitcond, 'SOFTSTOP')
       app.ControllerResponseTextArea.Value = ...
           'Soft stop completed: motion ramped down and halted.';
   end
   ```

   **Call site B** — inside the `JogPlatform(app, jogvec)` helper method (around line 209), currently:
   ```matlab
   if ~app.Emulated;
       StreamContourData_to_Galil(g, app.hex_path, 0);
   end
   ```
   Replace the whole `if ~app.Emulated ... end` block with:
   ```matlab
   app.softStopToken.reset();
   StreamContourData_to_Galil(g, app.hex_path, 0, 0, app.softStopToken);
   ```
   (Jogs are typically short, so reacting to Soft Stop here is a nice-to-have rather than critical. Updating both keeps the call-site pattern consistent.)

   Note: the `if ~app.Emulated` guards can *all* be deleted now — see the §4.3 sweep below.
7. Add the Soft Stop button callback:
   ```matlab
   function SoftStopButtonPushed(app, event)
       app.softStopToken.request();
       app.ControllerResponseTextArea.Value = ...
           'Soft stop requested; ramping down over next 1 s...';
   end
   ```
8. Save.

---

### §5.2 — Wire the Animate Motion button

**Backing function (already landed):** [../AnimateHexapod.m](../AnimateHexapod.m)

The button already exists (`AnimateMotionButton` on the Plan Motion tab) and already has a `ButtonPushedFcn` wired — it just has a commented-out body. Open its callback and replace the body.

**Current:**
```matlab
function AnimateMotionButtonPushed(app, event)
%             [vid_fig]=AnimateHexapod(app,hex_obj,hex_setup,hex_path)
end
```

**Replace with:**
```matlab
function AnimateMotionButtonPushed(app, event)
    if ~isfield(app.hex_path, 'pose_t') || isempty(app.hex_path.pose_t)
        uialert(app.UIFigure, ...
            'No motion profile loaded. Generate or import a path first.', ...
            'Animate Motion');
        return
    end
    AnimateHexapod(app.hex_obj, app.hex_setup, app.hex_path);
end
```

No new properties needed. Save.

**What you get:** a separate figure window showing the hexapod rendered in 3D, with a bottom control panel containing:
- `|<<` rewind to start, **Play / Pause** toggle (bold), `>>|` jump to end
- Scrubbable time slider — drag to any frame; playback pauses on scrub
- Speed popup: 0.1× / 0.25× / 0.5× / 1× (default) / 2× / 4× / 10×
- Loop checkbox — when ticked, playback wraps from end back to start instead of stopping
- Time readout showing `elapsed / total` seconds and `frame / N`

Closing the figure stops and deletes its timer automatically. Multiple Animate windows can coexist — each owns its own timer and state. Playback uses `hexapodGraphic_nocalc_replot` to update graphics in-place (no delete/recreate per frame).

**Smoke test:**
1. Launch HexControl (emulator mode is fine).
2. Initialize Controller.
3. Press **Home and Generate Path** (or load a TestMotion profile via Import).
4. Press **Animate Motion** → a new figure should open with the hexapod at the starting pose.
5. Press **Play** → hexapod moves through the trajectory at real time.
6. Drag the slider → pose tracks immediately; playback pauses.
7. Change speed to 4× → motion plays 4× faster when you hit Play again.
8. Tick Loop → playback wraps instead of stopping at the end.
9. Close the figure → no lingering timers (`timerfind` in the Command Window should show nothing belonging to this animation).

**Known limitations (deferred):**
- Runs `InverseKinematics_hexapod` per frame rather than consuming the pre-computed `hex_path.plati` / `hex_path.lhat` tensors. For typical TestMotion lengths the per-frame IK cost is imperceptible, but for very long paths or on an older machine, pre-caching the geometry once up front is an obvious follow-up optimization.

---

### Live encoder DRO feedback from the controller

**Backing code (already landed):**
- [../EncoderToLength.m](../EncoderToLength.m) — inverse of `LengthToEncoder.m`.
- [../updateEncoderDROs.m](../updateEncoderDROs.m) — single-shot updater that polls `TP` and writes into the `ActuatorPositions` cell.

**What this adds:** a 10 Hz timer that, while the app is connected, polls the controller for encoder counts and pushes them into the `AxNumeric_1..AxNumeric_6` readouts plus the `Ax1Gauge..Ax6Gauge` linear gauges. Works identically against the real controller and the emulator (both expose `GCommand('TP')`). Platform pose (`DRO_X/Y/Z/Rx/Ry/Rz`) and joint-separation fields are *not* touched by this — they still update on jog/execute via `updateGauges`, and a proper digital-shadow feed for them is scoped in §5.3.

**Steps in App Designer:**

1. Add a new property in Properties (Access = public):
   ```matlab
   encoderTimer    % timer handle polling TP at 10 Hz
   ```

2. In `InitializeControllerButtonPushed`, right **after** the `app.Connected = 1;` line at the end of the successful-connect `try` block, start the timer:
   ```matlab
       app.Connected = 1;

       % Stop any prior polling timer, then start a fresh one.
       try
           if ~isempty(app.encoderTimer) && isvalid(app.encoderTimer)
               stop(app.encoderTimer);
               delete(app.encoderTimer);
           end
       catch
       end
       app.encoderTimer = timer( ...
           'ExecutionMode', 'fixedSpacing', ...
           'Period', 0.1, ...           % 10 Hz
           'BusyMode',   'drop', ...
           'TimerFcn',   @(~,~) updateEncoderDROs(app.g, app.hex_setup, app.ActuatorPositions));
       start(app.encoderTimer);
   ```

3. Also stop the timer on controller-init failure. Inside the `catch ME` block, before the final `UpdateLEDs(app);`, add:
   ```matlab
       try
           if ~isempty(app.encoderTimer) && isvalid(app.encoderTimer)
               stop(app.encoderTimer);
               delete(app.encoderTimer);
           end
       catch
       end
       app.encoderTimer = [];
   ```

4. Clean up on app close. In Design View, select the main `UIFigure`. In the Component Browser → **Callbacks** tab, add a `UIFigureCloseRequest` callback, and paste:
   ```matlab
   function UIFigureCloseRequest(app, event)
       try
           if ~isempty(app.encoderTimer) && isvalid(app.encoderTimer)
               stop(app.encoderTimer);
               delete(app.encoderTimer);
           end
       catch
       end
       try
           if ~isempty(app.g)
               app.g.GClose;
           end
       catch
       end
       delete(app);
   end
   ```
   (The `app.g.GClose` tidies up any controller connection too. If a `UIFigureCloseRequest` already exists, fold the two `try` blocks into its top — don't duplicate the whole function.)

5. Save.

**Smoke test (emulator mode):**
1. Initialize Controller → encoder gauges sit at zero.
2. Press any jog button (with Jog Distance ≥ 1 mm). As the emulator integrates the commanded increments through its virtual contour buffer, `AxNumeric_1..6` and the linear gauges should tick up smoothly over the ~1 s the motion takes, instead of snapping to the final value.
3. Press Execute Motion on a long profile → gauges animate throughout the run.
4. Close the app window → in the Command Window, `timerfindall` should report no lingering timers whose name matches this app.

**Tuning:**
- Period `0.1` (10 Hz) is a reasonable starting point. Lower it (e.g. `0.033` for 30 Hz) if the DROs feel laggy; raise it (`0.2` for 5 Hz) if you want less Galil chatter.
- `BusyMode='drop'` means a slow network round-trip to the real controller will skip the next tick rather than queue up — keeps the UI responsive.
- Nothing polls when the timer is stopped; no overhead when you're not connected.

---

### §5.3 — Upgrade the encoder-feedback timer to a full digital shadow

**Backing code (already landed):**
- [../ForwardKinematics_hexapod.m](../ForwardKinematics_hexapod.m) — Newton-Raphson solver, numerical Jacobian, warm-start friendly.
- [../updateDigitalShadow.m](../updateDigitalShadow.m) — drop-in replacement for `updateEncoderDROs` that extends it with the FK solve + pose/joint-separation DRO updates.

**What this adds on top of §"Live encoder DRO feedback":** instead of only writing encoder counts and actuator lengths, the 10 Hz timer now also solves forward kinematics on each tick and writes `DRO_X/Y/Z`, `DRO_Rx/Ry/Rz`, and the `AB1..6` / `CD1..6` joint-clearance fields. Result: *every* DRO widget on the main panel tracks real (or emulated) state in real time, not just the actuator-length half.

**Steps in App Designer:** one line. In `InitializeControllerButtonPushed`, find the timer-creation block you added in the §"Live encoder DRO feedback" step. Change only the `TimerFcn` line:

Before:
```matlab
app.encoderTimer = timer( ...
    'ExecutionMode', 'fixedSpacing', ...
    'Period', 0.1, ...
    'BusyMode',   'drop', ...
    'TimerFcn',   @(~,~) updateEncoderDROs(app.g, app.hex_setup, app.ActuatorPositions));
```

After (pass the whole `app` handle so `r_rel` / `Home` changes from the Coordinate System tab are picked up on every tick):
```matlab
app.encoderTimer = timer( ...
    'ExecutionMode', 'fixedSpacing', ...
    'Period', 0.1, ...
    'BusyMode',   'drop', ...
    'TimerFcn',   @(~,~) updateDigitalShadow(app));
```

Save.

**Why pass the `app` handle instead of individual fields:** the FK solve itself only depends on the fixed mechanical geometry (`plat_link_0`, `base_link`), which doesn't change at runtime. But the display-frame transform from platform pose to end-effector pose — `disp_xyz = plat_CM + R*r_rel - Home` — uses `hex_obj.r_rel` and `hex_obj.Home`, both of which the user can change via the Coordinate System tab's Enter button. Capturing individual fields in the timer closure would snapshot those values at Initialize time and miss later changes. Taking the `app` handle reads them fresh each tick. No re-Initialize needed after adjusting the CG offset.

**Behavior:**
- **Platform-space solve, display-frame output:** FK returns platform pose (plat_CM + Euler); the updater applies the current `r_rel` / `Home` to get the end-effector pose for the DROs. Changing CG offset shifts the DROs live without re-solving.
- **Warm start:** the `persistent pose_seed_plat` inside the updater carries across ticks (in platform-pose space, so `r_rel` changes don't invalidate it). During smooth motion, Newton converges in 1–2 iterations; test suite confirms < 3 iters/step average.
- **Cold start:** the first tick after app launch seeds at zero platform pose. As long as the hexapod isn't dramatically offset when Initialize is pressed, convergence is fine; the first tick may need more iterations, subsequent ticks settle into the warm-start rhythm.
- **Graceful failure:** if the Jacobian goes singular near mechanical limits or the solver doesn't converge in 8 iterations, pose and joint-separation widgets are left at their previous values (encoder / length widgets still update). The seed is *not* advanced, so the next tick retries from the last good platform pose.

**Smoke test (emulator mode):**
1. Launch HexControl → Initialize Controller. All DROs should sit at zero; no errors.
2. Jog +X by 10 mm. You should see:
   - `AxNumeric_1..6` and linear gauges animate through the motion (as before).
   - `DRO_X` tick up smoothly from 0 to ~0.010 m.
   - `AB1..6` / `CD1..6` joint clearances shift as the geometry changes.
3. Press **Return All Actuators to Zero**. Everything returns to zero.
4. Load a TestMotion profile, press Execute. All DROs animate through the trajectory.

**Unit tests:** run [../tests/test_ForwardKinematics.m](../tests/test_ForwardKinematics.m). Exercises round-trip `X → IK → L → FK → X'` on nine representative poses (cold-start) plus a 20-step warm-start sequence; all should pass within 1e-8 m / rad per DOF.

**Revert:** to go back to encoder-only feedback (e.g. if a hardware fault makes FK unstable), edit the `TimerFcn` line back to the `updateEncoderDROs` version. No other changes needed.

---

### Soft-stop lockout + Return-to-Datum + Datum frame

Follow-up to §5.4 soft-stop and the Datum-vs-Home architectural split. Three related pieces, all App-Designer-side.

**Backing code (landed):**
- [../InitializeHexapodObject.m](../InitializeHexapodObject.m) replaces the old `Home` / `Home_platform` / `Datum` / `Datum_platform` / `r_rel` scalar fields with two rigid-body transforms and one fixed reference point:
  - `hex_obj.T_world_datum_platform` — struct `{R, t}` specifying where the platform-CM sits in the world frame at pose = 0. User-settable.
  - `hex_obj.T_platform_POI` — struct `{R, t}` specifying the POI's pose in the platform body frame (translation *and* rotation). User-settable.
  - `hex_obj.Home_platform` — immutable 3x1 vector: physical platform-CM world position at hardware re-home.
- Motion commands are now `T_datum_POI` (the POI's pose relative to the datum frame) encoded as the familiar 6-vector `hex_obj.pose = [x; y; z; roll; pitch; yaw]`. Euler axes are the datum frame's axes at pose = 0.
- IK, FK, graphics, simulation, digital shadow, checkerboard, and effector-from-platform helpers all compose frames via [../composeTransform.m](../composeTransform.m), [../invertTransform.m](../invertTransform.m), [../poseToTransform.m](../poseToTransform.m), [../transformToPose.m](../transformToPose.m), [../R2E.m](../R2E.m). All refactored to use the chain `T_world_POI = T_world_datum_platform * T_platform_POI * T_datum_POI` (and its inverse for display).
- At default initialization both transforms are identity rotations, so kinematics output is bitwise identical to pre-refactor behavior for a fresh app. Rotations only "light up" when the user sets them via the new Datum / POI controls.
- [../ReturnToDatumPath.m](../ReturnToDatumPath.m) — generates a smooth 2 s S-curve trajectory from current pose to `[0;0;0;0;0;0]` (datum origin). Unchanged.

**Steps in App Designer:**

1. **Add a new property** in Properties (Access = public):
   ```matlab
   softStopLockout   logical = false    % set on SOFTSTOP; cleared after Return to Datum completes
   ```

2. **Guard `ExecuteMotionButtonPushed`**. At the top of the callback, before anything else:
   ```matlab
   if app.softStopLockout
       uialert(app.UIFigure, ...
           'Motion execution is locked out after a soft stop. Press "Return to Datum" first.', ...
           'Soft-stop lockout', 'Icon', 'warning');
       return
   end
   ```
   Then at the end of the callback, AFTER `StreamContourData_to_Galil` returns, check the exit condition and latch the flag:
   ```matlab
   if strcmp(exitcond, 'SOFTSTOP')
       app.softStopLockout = true;
       app.ControllerResponseTextArea.Value = ...
           'Soft-stop engaged. Press "Return to Datum" before further motion.';
   end
   ```

3. **Do NOT guard `JogPlatform`**. Jogs remain available after a soft-stop — they're the primary way a user walks the platform back toward datum manually, inspects it at intermediate positions, and clears whatever triggered the soft-stop. Only Execute Motion (running an imported/generated contour) is locked out until an explicit "Return to Datum".

4. **Repurpose `ReturnAllActuatorstoZeroButton` → Return to Datum.**
   - In Design View, select the button. Change its **Text** to `Return to Datum`. The Name property can stay `ReturnAllActuatorstoZeroButton` to avoid reworking the callback signature; rename to `ReturnToDatumButton` if you prefer (and update the callback name correspondingly).
   - Replace the callback body. It mirrors `JogPlatform` structurally — plan a path, simulate, check collisions, stream, update displayed state — just with `ReturnToDatumPath` in place of `JogFromCurrentPosition` so the target pose is `[0;0;0;0;0;0]` (datum origin):
     ```matlab
     function ReturnAllActuatorstoZeroButtonPushed(app, event)
         if ~app.Connected
             msgbox('Not connected to controller');
             return
         end
         g = app.g;

         % Plan a smooth S-curve from current pose to datum origin.
         app.hex_path = ReturnToDatumPath(app.hex_obj, app.hex_path);
         app.hex_path = SimulateMotionProfile_kinematics(app.hex_obj, app.hex_setup, app.hex_path);

         if max(app.hex_path.collisioncheck)
             msgbox('U-joint collision predicted along return path; cannot return to datum.');
             return
         end

         app.softStopToken.reset();
         StreamContourData_to_Galil(g, app.hex_path, 0, 0, app.softStopToken);

         % Same post-motion updates as a jog completion.
         app.hex_obj.pose = app.hex_path.pose_t(:,end);
         app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
         app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
         app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);

         % Lockout cleared on successful return - Execute Motion re-enabled.
         app.softStopLockout = false;
         app.ControllerResponseTextArea.Value = 'At datum. Motion execution re-enabled.';
     end
     ```
   - This callback intentionally does *not* check `softStopLockout` — it's the way OUT of the lockout, so it must remain available.

5. **Rework the Coordinate System `EnterButtonPushed` callback for the POI-transform field.** The old fields (`r_rel`, `Home`) are gone. The Coordinate System tab's X / Y / Z mm fields now update the *translation* part of `T_platform_POI`:
   ```matlab
   app.hex_obj.T_platform_POI.t = [xoffset; yoffset; zoffset];
   app.hex_obj = EffectorFromPlatformPose(app.hex_obj);  % recompute display pose
   app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
   app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
   app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);
   ```
   (Delete the `app.hex_obj.r_rel = ...` and `app.hex_obj.Home = ...` lines — those fields no longer exist.)

   **Why the platform stays put:** `T_platform_POI` changes → `T_world_datum = T_world_datum_platform * T_platform_POI` shifts by the same amount. At pose = 0 the platform CM is still at `T_world_datum_platform.t` because that's what anchors the chain. Old r_rel-preservation behavior, structurally explicit now.

   If you later want rotation support (mount a test article canted), expose three more fields on the Coordinate System tab for Euler angles of the POI relative to the platform body, and set `app.hex_obj.T_platform_POI.R = E2R([roll; pitch; yaw])`. No other code change needed; IK / FK / graphics already handle non-identity POI rotations through the frame chain.

6. **Add Datum controls.** Two new buttons on the Coordinate System tab (numeric Datum offset fields can come later if needed).

   **Button A: "Set Datum Here"** — captures the current platform pose as the new datum-platform anchor, so motion planning re-centers around wherever the hexapod currently is.
   ```matlab
   function SetDatumHereButtonPushed(app, event)
       % Current platform world pose = pose_platform from the last IK.
       app.hex_obj.T_world_datum_platform = ...
           poseToTransform(app.hex_obj.pose_platform);
       % Pose becomes [0;0;0;0;0;0] by definition (we're at the datum).
       app.hex_obj.pose = zeros(6,1);
       app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
       app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
       app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);
   end
   ```

   **Button B: "Reset Datum to Home"** — anchors the datum-platform back at the physical Home (translation only; orientation identity).
   ```matlab
   function ResetDatumToHomeButtonPushed(app, event)
       % Rebuild T_world_datum_platform at identity rotation and
       % Home_platform translation.
       app.hex_obj.T_world_datum_platform = struct( ...
           'R', eye(3), ...
           't', app.hex_obj.Home_platform);
       % Platform hasn't physically moved; recompute its display pose in
       % the reset datum frame.
       app.hex_obj = EffectorFromPlatformPose(app.hex_obj);
       app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
       app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
       app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);
   end
   ```

   If you want the datum to also support rotation ("plan motions in a frame tilted 15 deg about Y relative to Home"), either (a) add roll/pitch/yaw fields to the tab, applying `E2R([r;p;y])` into `T_world_datum_platform.R`, or (b) set the rotation via "Set Datum Here" while the platform is at a tilted pose. Both are now supported by the math.

7. **Fix the `ReHomeActuatorsButtonPushed` display pose update.** Currently it sets `app.hex_obj.pose = [0,0,0,0,0,0]'` after HM, which was only correct when the datum coincided with physical home. Under the general frame model, after HM the platform is physically at `Home_platform` with identity orientation, so the display pose should reflect that through the frame chain:
   ```matlab
   app.hex_obj.pose = [0,0,0,0,0,0]';
   ```
   Replace with:
   ```matlab
   % Platform is physically at Home_platform after HM.
   app.hex_obj.pose_platform = [app.hex_obj.Home_platform; 0; 0; 0];
   app.hex_obj = EffectorFromPlatformPose(app.hex_obj);
   ```
   (When the datum-platform is at its default — identity rotation at `Home_platform` — and `T_platform_POI` is identity, `EffectorFromPlatformPose` yields `pose = [0;0;0;0;0;0]`, matching old behavior. When the user has moved the datum or set a non-identity POI offset, the display pose reflects those transforms correctly.)

8. **Save.**

**Smoke test:**
- Launch, initialize, jog +Z by 10 mm. DROs should show `Z = 0.010 m`. (Default Datum = Home path still reads zero before the jog.)
- Press **Set Datum Here**. DROs should jump to all zeros (we just defined this as the new datum).
- Jog +Z 10 mm again. DROs show `Z = 0.010 m`.
- Press **Return to Datum**. Platform returns to the set datum; DROs read zero.
- Press **Reset Datum to Home**. DROs should jump to show the offset between the previously-set datum and home.
- Press **Return to Datum** again — platform returns to Home (since Datum is now Home).
- Run any long motion, press **Soft Stop** mid-run — `softStopLockout` latches. Execute Motion attempts now pop the warning dialog, but jog buttons still work (use them to inspect or manually nudge). Press **Return to Datum** — platform drives smoothly back to `[0;0;0;0;0;0]` in the datum frame, lockout clears, Execute Motion re-enabled.

---

### §4.3 — Delete all remaining `if ~app.Emulated` guards

Now that [galilFactory](../Emulator/galilFactory.m) returns a live [GalilEmulator](../Emulator/GalilEmulator.m) that implements the same API surface as `py.gclib.py`, every callback can call `g.GCommand(...)` uniformly. The old `if ~app.Emulated` wrappers only existed because the codebase used to have no stand-in for the hardware.

**Callbacks that still contain `if ~app.Emulated` — drop the wrapper in each:**

| Callback | Lines to delete | Lines to keep |
|---|---|---|
| `ExecuteMotionButtonPushed` | `if ~app.Emulated;` and its matching `end` | The `StreamContourData_to_Galil(...)` call inside (already updated with the soft-stop token in §5.4 above) |
| `JogPlatform` | `if ~app.Emulated;` and its matching `end` | The `StreamContourData_to_Galil(...)` call inside |
| `ReturnAllActuatorstoZeroButtonPushed` | `if ~app.Emulated;` and its matching `end` | The four `g.GCommand(...)` lines |
| `ReHomeActuatorsButtonPushed` | `if ~app.Emulated;` and its matching `end` (inside the `'Yes'` case) | The three `g.GCommand(...)` lines |
| `UNLOCKBrakesButtonPushed` | `if ~app.Emulated` and its matching `end` | `g=app.g; g.GCommand('SHABCEFG');` |
| `LOCKBrakesButtonPushed` | `if ~app.Emulated` and its matching `end` | `g=app.g; g.GCommand('ST'); g.GCommand('MO');` |

The outer `if(app.Connected) ... else msgbox('Not connected...') end` guards in these callbacks are still meaningful — keep them.

**Update `EmulateGalilConnectionforDebuggingCheckBoxValueChanged`:**

Currently:
```matlab
function EmulateGalilConnectionforDebuggingCheckBoxValueChanged(app, event)
    value = app.EmulateGalilConnectionforDebuggingCheckBox.Value;
    app.Emulated = value;
    app.Connected = 1;   % old shortcut -- bypasses Initialize Controller
end
```

The old `app.Connected = 1` shortcut let callbacks pass their `if(app.Connected)` guard without the user ever pressing **Initialize Controller** — which was fine when the `if ~app.Emulated` branches then skipped all the `g.GCommand` work. With those branches removed, `app.g` is still the initial `0` at that point and any `g.GCommand` call would explode. Replace with:

```matlab
function EmulateGalilConnectionforDebuggingCheckBoxValueChanged(app, event)
    app.Emulated = app.EmulateGalilConnectionforDebuggingCheckBox.Value;
    % Toggling emulation invalidates any previous controller handle.
    % Press Initialize Controller to bind a fresh handle (emulator or
    % real) via galilFactory.
    app.Connected = 0;
    app.LED_stat.ControllerInitialized = 0;
    UpdateLEDs(app);
end
```

**New workflow:** tick Emulate → press **Initialize Controller** → green LED → every other button "just works" against the emulator. Flip the checkbox off → press Init again → same buttons now drive the real controller.

**Acceptance:** grep the `.mlapp`'s extracted source for `if ~app.Emulated` — zero hits. Full motion / jog / brake / home workflow runs end-to-end against the emulator without errors and without `g = 0` exceptions.

**Smoke test (against emulator, per §5.1 wire-up):**
- Load a long motion (≥3 s).
- Press **Execute Motion**; after ~1 s, press **Soft Stop**.
- The encoder-trace in the response area should show smooth deceleration over ~1 s; final positions settle instead of snapping. No errors.

**Unit tests:** [../tests/test_softstop.m](../tests/test_softstop.m) exercises the taper path against the emulator with an immediate cancel.

---

## Latest update — three-frame refactor delta

Only the four callback bodies below need to change versus what you previously applied. Everything else from the "Soft-stop lockout + Return-to-Datum + Datum frame" section above (the `softStopLockout` property, the Execute Motion guard, the lack-of-a-jog-guard, the "Return to Datum" button repurposing) stays exactly as you already wired it. This delta covers only the four callbacks that were touching the now-retired `Home` / `Datum` / `Datum_platform` / `r_rel` fields.

The new fields, for reference (fully described in [../InitializeHexapodObject.m](../InitializeHexapodObject.m)):
- `hex_obj.T_world_datum_platform` — struct `{R, t}`, platform-CM world pose at pose = 0. User-settable.
- `hex_obj.T_platform_POI` — struct `{R, t}`, POI pose in platform body frame. User-settable.
- `hex_obj.Home_platform` — immutable 3×1 vector, physical re-homing platform-CM world position.

---

### Delta 1 — Coordinate System `EnterButtonPushed`

**Old body (replace entirely):**
```matlab
app.hex_obj.r_rel = [xoffset; yoffset; zoffset];
app.hex_obj.Datum = app.hex_obj.Datum_platform + app.hex_obj.r_rel;
app.hex_obj = EffectorFromPlatformPose(app.hex_obj);
app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);
```

**New body:**
```matlab
xoffset = app.XmmEditField.Value / 1000;
yoffset = app.YmmEditField.Value / 1000;
zoffset = app.ZmmEditField.Value / 1000;

app.hex_obj.T_platform_POI.t = [xoffset; yoffset; zoffset];
app.hex_obj = EffectorFromPlatformPose(app.hex_obj);
app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);
```

Net change: one assignment (`T_platform_POI.t` in, `r_rel` + `Datum` out). The `EffectorFromPlatformPose` call now does the frame-chain work internally.

---

### Delta 2 — "Set Datum Here" callback

**Old body:**
```matlab
current_r_world = app.hex_obj.pose(1:3) + app.hex_obj.Datum;
app.hex_obj.Datum = current_r_world;
app.hex_obj.Datum_platform = app.hex_obj.Datum - app.hex_obj.r_rel;
app.hex_obj.pose(1:3) = [0;0;0];
app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);
```

**New body:**
```matlab
% Take the current platform pose (world frame) as the new datum anchor.
app.hex_obj.T_world_datum_platform = poseToTransform(app.hex_obj.pose_platform);
app.hex_obj.pose = zeros(6,1);    % at the new datum by definition
app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);
```

---

### Delta 3 — "Reset Datum to Home" callback

**Old body:**
```matlab
current_r_world = app.hex_obj.pose(1:3) + app.hex_obj.Datum;
app.hex_obj.Datum_platform = app.hex_obj.Home_platform;
app.hex_obj.Datum          = app.hex_obj.Home_platform + app.hex_obj.r_rel;
app.hex_obj.pose(1:3) = current_r_world - app.hex_obj.Datum;
app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);
```

**New body:**
```matlab
app.hex_obj.T_world_datum_platform = struct( ...
    'R', eye(3), ...
    't', app.hex_obj.Home_platform);
app.hex_obj = EffectorFromPlatformPose(app.hex_obj);
app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);
```

---

### Delta 4 — `ReHomeActuatorsButtonPushed` pose update

Inside the existing callback, find the two lines that set the display pose after HM and replace them.

**Old (inside the `case 'Yes'` branch, after the three `g.GCommand` calls):**
```matlab
at_home_world = app.hex_obj.Home_platform + app.hex_obj.r_rel;
app.hex_obj.pose = [at_home_world - app.hex_obj.Datum; 0; 0; 0];
```

**New:**
```matlab
% Platform is physically at Home_platform after HM, identity orientation.
app.hex_obj.pose_platform = [app.hex_obj.Home_platform; 0; 0; 0];
app.hex_obj = EffectorFromPlatformPose(app.hex_obj);
```

Everything below these lines in `ReHomeActuatorsButtonPushed` — the IK, replot, update-gauges — stays as-is.

---

### After applying

Smoke test same as before: jog, press **Set Datum Here**, verify DROs zero out; jog again, press **Return to Datum**, verify return; press **Reset Datum to Home**, verify the DROs reflect the offset between your custom datum and home. If anything looks off, scroll up to the full "Soft-stop lockout + Return-to-Datum + Datum frame" section for the wider context; this delta assumes the rest of that section is already in place.

---

## Latest update — async streaming delta

Streaming to the Galil has been moved off the calling thread onto a MATLAB timer inside [../ContourStreamSession.m](../ContourStreamSession.m). The old `StreamContourData_to_Galil.m` is now a *synchronous wrapper* around that session (pauses 0.05 s and polls `session.isDone()`), so every existing call site still works byte-compatibly. But for app callbacks, switching to the direct session API removes the last place the UI thread sat in a blocking drawnow loop — encoder DROs, animation timers, and Soft Stop clicks now fire at their full scheduled rate instead of sharing cycles with a streaming loop.

**Why this matters:**
Before, each streaming iteration spent most of its wall time inside `drawnow` yielding to other timers — on a loaded machine those yields occasionally stretched past the ~200 ms it takes the Galil contour buffer (511 slots, DT-governed drain rate) to starve. With the session-timer model, the streaming tick itself is a ~1-2 ms chunk-send + CM? query, and everything else runs between ticks without contention. Buffer starvation risk drops correspondingly, and the `buffer_health.min_pending` field in the returned record makes it observable if it ever approaches the threshold.

**When to use which API:**
- App callbacks (`ExecuteMotionButtonPushed`, `JogPlatform`, `ReturnAllActuatorstoZero...`) → use `ContourStreamSession` directly, with an `on_complete` handler. **Callback returns immediately.**
- Scripts / `test_softstop` / `test_stream_integration` / anything outside the app → keep calling `StreamContourData_to_Galil(...)`. No change.

### Delta 5 — `ExecuteMotionButtonPushed` async body

Replace the block that currently reads:
```matlab
app.softStopToken.reset();
[exitcond, ~] = StreamContourData_to_Galil( ...
    g, app.hex_path, 1, 0, app.softStopToken);
if strcmp(exitcond, 'SOFTSTOP')
    app.softStopLockout = true;
    app.ControllerResponseTextArea.Value = ...
        'Soft-stop engaged. Press "Return to Datum" before further motion.';
end
```
with:
```matlab
app.softStopToken.reset();

% Fire-and-forget: session streams on its own timer and invokes
% onExecuteMotionComplete on finish. This callback returns now.
app.currentStreamSession = ContourStreamSession( ...
    g, app.hex_path, 1, 0, app.softStopToken, ...
    @(exitcond, record) onExecuteMotionComplete(app, exitcond, record));
```

Add a new property in Properties (Access = private):
```matlab
currentStreamSession   % ContourStreamSession handle, or []
```

Add a private helper method at the bottom of the class (`methods (Access = private)`):
```matlab
function onExecuteMotionComplete(app, exitcond, record)
    % Called from the session's timer thread context when streaming
    % finishes. UI updates must go through uifigure-safe paths; direct
    % widget writes are fine from a timer callback.
    if strcmp(exitcond, 'SOFTSTOP')
        app.softStopLockout = true;
        app.ControllerResponseTextArea.Value = ...
            'Soft-stop engaged. Press "Return to Datum" before further motion.';
    else
        app.ControllerResponseTextArea.Value = 'Motion complete.';
    end
    if isfield(record, 'buffer_health') && record.buffer_health.starved_risk
        app.ControllerResponseTextArea.Value = ...
            sprintf('%s (Buffer dropped to %d samples — close to starvation.)', ...
                    app.ControllerResponseTextArea.Value, ...
                    record.buffer_health.min_pending);
    end
    app.currentStreamSession = [];   % release the handle for GC
end
```

### Delta 6 — `JogPlatform(app, jogvec)` async body

Currently:
```matlab
app.softStopToken.reset();
StreamContourData_to_Galil(g, app.hex_path, 0, 0, app.softStopToken);
```
Becomes:
```matlab
app.softStopToken.reset();
app.currentStreamSession = ContourStreamSession( ...
    g, app.hex_path, 0, 0, app.softStopToken, ...
    @(exitcond, ~) onJogComplete(app, exitcond));
```

And a private helper:
```matlab
function onJogComplete(app, exitcond) %#ok<INUSD>
    % Jogs don't latch the lockout on SOFTSTOP (soft-stop during a jog
    % is uncommon and jogs remain enabled after a lockout anyway).
    app.currentStreamSession = [];
end
```

**Important:** do NOT move the "update hex_obj.pose / replot / update gauges" post-motion block out of `JogPlatform` and into `onJogComplete`. The existing post-motion block sets the *commanded* pose, not measured — it's computed from `app.hex_path.pose_t(:,end)`, which is known before streaming starts. Leave it right after the `ContourStreamSession` call so the UI updates immediately with the target. The digital-shadow timer is what reconciles display with actual measured encoder state; that happens on its own cadence regardless of when streaming completes.

### Delta 7 — `ReturnAllActuatorstoZeroButtonPushed` async body

Currently (from Delta "Repurpose" section above):
```matlab
app.softStopToken.reset();
StreamContourData_to_Galil(g, app.hex_path, 0, 0, app.softStopToken);

% Same post-motion updates as a jog completion.
app.hex_obj.pose = app.hex_path.pose_t(:,end);
app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);

% Lockout cleared on successful return - Execute Motion re-enabled.
app.softStopLockout = false;
app.ControllerResponseTextArea.Value = 'At datum. Motion execution re-enabled.';
```
Becomes:
```matlab
app.softStopToken.reset();
app.currentStreamSession = ContourStreamSession( ...
    g, app.hex_path, 0, 0, app.softStopToken, ...
    @(exitcond, ~) onReturnToDatumComplete(app, exitcond));

% Commanded-pose UI updates can happen right now — they target the
% *final* pose of the planned return. Measured reconciliation is the
% digital shadow's job.
app.hex_obj.pose = app.hex_path.pose_t(:,end);
app.hex_obj = InverseKinematics_hexapod(app.hex_obj, app.hex_setup);
app.hex_plot3D = hexapodGraphic_nocalc_replot(app.hex_obj, app.hex_plot3D);
app.ActuatorPositions = updateGauges(app.hex_obj, app.ActuatorPositions);
```

And the completion handler — note the lockout clears ONLY on a clean `DONE`; a soft-stop mid-return leaves the lockout latched (the return itself got interrupted, so we haven't returned to datum):
```matlab
function onReturnToDatumComplete(app, exitcond)
    if strcmp(exitcond, 'DONE')
        app.softStopLockout = false;
        app.ControllerResponseTextArea.Value = 'At datum. Motion execution re-enabled.';
    elseif strcmp(exitcond, 'SOFTSTOP')
        % Return itself was soft-stopped. Lockout stays.
        app.ControllerResponseTextArea.Value = ...
            'Return-to-Datum soft-stopped. Press Return to Datum again to resume.';
    end
    app.currentStreamSession = [];
end
```

### Delta 8 — guard against double-launch

Because callbacks now return before streaming finishes, a fast clicker could press Execute Motion twice before the first run completes. Add a guard at the top of every callback that launches a session:
```matlab
if ~isempty(app.currentStreamSession) && ~app.currentStreamSession.isDone()
    uialert(app.UIFigure, ...
        'A motion is already running. Wait for completion or press Soft Stop.', ...
        'Streaming in progress', 'Icon', 'warning');
    return
end
```
Apply this inside `ExecuteMotionButtonPushed`, `JogPlatform`, and `ReturnAllActuatorstoZeroButtonPushed`, above the `softStopToken.reset()` call.

### Delta 9 — clean up in `UIFigureCloseRequest`

So the background timer doesn't outlive the app window, stop any live session when the user closes the figure:
```matlab
if ~isempty(app.currentStreamSession) && isvalid(app.currentStreamSession)
    delete(app.currentStreamSession);   % triggers stopTimer via delete()
end
```
Put this at the top of `UIFigureCloseRequest`, before any other cleanup (timers, `g.GClose`, etc.).

### After applying

Smoke tests:
- **Responsiveness during motion:** Run a long (30 s+) Execute Motion. While it streams, press jog buttons (they'll queue with the double-launch guard showing the dialog — correct behavior), but notice that the encoder DROs, animation, and axis gauges *all keep ticking at full rate* throughout. Pre-refactor these froze during streaming.
- **Soft Stop responsiveness:** Start a long motion, click Soft Stop. The Hann taper begins inside ~50 ms (one tick period) — pre-refactor latency was whatever drawnow happened to yield at, sometimes 200+ ms.
- **Buffer-health surfaced:** After any motion, the `ControllerResponseTextArea` prints `min_pending` if it dropped below 50. If it never prints the warning, you have comfortable headroom.
- **Clean shutdown:** Close the app window mid-stream. No orphan timers should remain — verify with `timerfind` at the command line.
- **Script/test backward compat:** `test_softstop` and `test_stream_integration` must still pass unchanged — they drive `StreamContourData_to_Galil`, now via the sync wrapper.

---

## Completed

*(Nothing yet — the UI changes above still need to be applied inside App Designer.)*
