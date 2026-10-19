# Debugging the drivetrain (wild spin on enable)

Use this when the robot spins or behaves erratically when you enable.

---

## Fixes applied (try these first)

1. **CANBus no longer loads Hoot replay on real hardware** — In `TunerConstants.java`, `kCANBus` was `new CANBus("", "./logs/example.hoot")`, which can load a replay file and interfere with live CAN. It is now `new CANBus("")` so only the real CAN bus is used. If your build fails (e.g. no one-arg constructor), use `new CANBus("", null)` or restore the second arg for replay only when needed.

2. **HootAutoReplay only in simulation** — In `Robot.robotPeriodic()`, `m_timeAndJoystickReplay.update()` now runs only when `RobotBase.isSimulation()` is true. On the real robot, replay is not run, so it cannot override live joystick or CAN.

3. **Force Idle for 2 seconds after enable** — In `RobotContainer`, `kForceIdleSeconds = 2.0` and `Robot.teleopPeriodic()` calls `forceDriveIdleForSettling()`, which sends **Idle** to the drivetrain for the first 2 seconds after teleop starts (overriding any command). So for 2 seconds the robot should not move. If it **still spins during those 2 seconds**, the cause is not our drive commands—it is swerve config (TunerConstants encoder offsets/inversions) or hardware. Set `kForceIdleSeconds = 0` to disable this and drive immediately after the 0.5 s settling.

---

## 1. Watch SmartDashboard while enabling

The code publishes **DriveDebug/** values every cycle when the default drive command runs:

| Key | Meaning |
|-----|--------|
| **DriveDebug/TimeSinceTeleop** | Seconds since teleop started. Should go 0 → 0.5 → 1 → … |
| **DriveDebug/Settling** | `true` = we're sending **Idle** (zero). Should be true for first 0.5 s. |
| **DriveDebug/JoyLY, JoyLX, JoyRX** | Raw joystick (after NaN guard). Should be ~0 when sticks centered. |
| **DriveDebug/ReqVX, ReqVY, ReqRot** | What we're asking the drivetrain for (m/s, m/s, rad/s). Should be 0 when sticks centered. |

**When it spins, check:**

- If **ReqVX, ReqVY, ReqRot** are all **0** but the robot still spins → the problem is **not** our command; it's likely **swerve config** (TunerConstants encoder offsets, inversions, CAN IDs) or hardware.
- If **ReqRot** (or VX/VY) is **non‑zero** with sticks centered → joystick drift, wrong port, or deadband too small. Try increasing `Deadband` / `RotDeadband` in RobotContainer or replacing the controller.
- If **Settling** stays true for a long time or **TimeSinceTeleop** is wrong → check that `Robot.teleopInit()` runs and sets `RobotContainer.s_teleopStartTimeSeconds = Timer.getFPGATimestamp()`.

---

## 2. Force Idle only (no joystick drive)

To prove whether the spin comes from our drive logic or from the swerve stack:

1. In **RobotContainer**, temporarily change the default command to **always** return Idle:
   - In `configureBindings()`, replace the default command body with:
   - `return s_idle;` (and comment out the rest of the lambda).
2. Deploy and enable. Sticks should do nothing; robot should get Idle every cycle.

- **If it still spins** → problem is elsewhere: TunerConstants (encoder offsets, inversions), CAN IDs, or hardware. No command is asking for motion.
- **If it stays still** → our normal drive logic (joystick, settling, or deadband) is the cause. Restore the default command and use the DriveDebug values to see which part is wrong.

---

## 3. Log with Advantage Scope / DataLogManager

You already have **DataLogManager.start()** in Robot. That writes to the log file on the robot (or in the project folder in sim).

1. Enable, let it spin (or not), then stop.
2. Open the log in **Advantage Scope**.
3. Plot:
   - **DriveDebug/ReqVX**, **ReqVY**, **ReqRot**
   - **DriveDebug/JoyLX**, **JoyLY**, **JoyRX**
   - Any CTRE/Phoenix 6 topics for swerve (module states, steer position, etc.) if available.

That shows exactly what was requested and when, so you can see if non‑zero requests line up with the spin.

---

## 4. Check swerve config (TunerConstants / hardware)

If the robot spins even when **ReqVX, ReqVY, ReqRot** are 0:

- **Encoder offsets** in **TunerConstants** (e.g. `kFrontLeftEncoderOffset`, …) must match your physical zero. Wrong offset → steer motors chase the wrong angle and can spin.
- **Steer / encoder inversions** (`kFrontLeftSteerMotorInverted`, `kFrontLeftEncoderInverted`, etc.) must match your wiring. One wrong inversion can make a module spin.
- **CAN IDs** in TunerConstants (drive, steer, encoder per module) must match your wiring. Wrong ID can drive the wrong motor.
- **CAN bus name** in TunerConstants: on a real robot the main bus is usually `""` (empty). If you use a different bus or replay (e.g. Hoot), confirm the bus name and that you're not replaying old data.

Re-run **Tuner** (or your config tool) to regenerate/verify module constants if needed.

---

## 5. Disable other subsystems that use the drivetrain

To rule out another command or subsystem:

- Temporarily comment out **AutoAim** binding (X button), **SysId** bindings (Back/Start + X/Y), and any other bindings that run commands requiring the drivetrain.
- Enable with only the default drive command and the debug output. If the spin stops, re‑enable bindings one by one to see which one causes it.

---

## Quick checklist

- [ ] **DriveDebug/ReqVX, ReqVY, ReqRot** when sticks are centered (should be 0 after settling).
- [ ] **DriveDebug/JoyLX, JoyLY, JoyRX** when sticks are centered (should be ~0).
- [ ] **Force Idle only** test: default command always returns Idle → does it still spin?
- [ ] **TunerConstants**: encoder offsets, inversions, CAN IDs, CAN bus.
- [ ] **Advantage Scope**: plot DriveDebug and swerve topics over time when it spins.
