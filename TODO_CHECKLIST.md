# TODO checklist — verify on real robot

Everything marked with `// TODO` in code is listed here so you can tick items off once you have the turret and robot.

---

## Controllers & bindings

- [ ] **RobotContainer** — Verify controller ports (0 = driver, 1 = operator) match Driver Station.
- [ ] **RobotContainer** — Confirm operator trigger thresholds (0.5); right = full shoot, left = pre-aim only.

---

## CAN IDs & wiring

- [ ] **Constants.Motors** — Verify every CAN ID (and each subsection) matches your actual wiring before first run.

---

## Flywheel

- [ ] **Constants.Flywheel** — Set actual gear ratio (motor rotations per flywheel rotation) so RPM setpoint is correct.
- [ ] **Constants.Flywheel** — Tune at-speed tolerance (RPM); too tight = never ready, too loose = shoots before up to speed.
- [ ] **ShooterAssembly** — Verify secondary flywheel invert (Opposed); flip if wheels fight.

---

## Hood

- [ ] **Constants.Hood** — Set CANcoder ID if used; -1 if only motor integrated encoder.
- [ ] **Constants.Hood** — Calibrate degrees of hood motion per encoder rotation (gear train).
- [ ] **Constants.Hood** — Set min hood angle (deg) on robot; verify encoder increases as hood raises.
- [ ] **Constants.Hood** — Set max hood angle (deg) to match mechanical stop.
- [ ] **ShooterAssembly** — Verify hood current limit (30 A) for Neo 550 and mechanism.
- [ ] **ShooterAssembly** — Calibrate hood encoder zero (known angle at power-up or set in REV Hardware Client).
- [ ] **ShooterAssembly** — Verify hood encoder direction: positive = hood angle increasing (raising); negate if reversed.

---

## Turret

- [ ] **Constants.Turret** — Calibrate turret yaw degrees per motor rotation (encoder, gear ratio).
- [ ] **Constants.Turret** — Set soft limits (deg, robot-relative) to match mechanical turret stops.
- [ ] **Constants.Turret** — Set turret allowed arc (kAllowedMinDeg, kAllowedMaxDeg) on real robot for dead zone.
- [ ] **ShooterAssembly** — Calibrate turret encoder zero (e.g. forward = 0°) and verify soft limits match mechanical stops.
- [ ] **ShooterAssembly** — Verify turret encoder: 0° = forward; positive direction matches kAllowedMin/Max and dead zone.

---

## Shooter pose & shot math

- [ ] **ShooterAssembly** — Confirm drivetrain.getState().Speeds is field-relative (vx forward, vy left); if robot-relative, rotate by heading before use in lead.
- [ ] **ShooterAssembly** — Set shooter exit height (m) above field floor so heightDiff is correct for hub/shuttle z.
- [ ] **Constants.ShotCalculator** — Tune at hub: reference distance, RPM, hood angle.
- [ ] **Constants.ShotCalculator** — Tune exponent so RPM vs distance matches real ball flight.
- [ ] **Constants.ShotCalculator** — Tune hood slope (deg/m) so hood angle matches real ball flight.
- [ ] **Constants.ShotCalculator** — Set nominal muzzle speed (m/s) so velocity lead (shoot-ahead) is accurate.
- [ ] **ShotCalculator** — Tune height scaling (2.0) at field for target higher than shooter exit.

---

## Index & tower

- [ ] **Constants.Index** — Tune shoot RPM on robot so note feeds reliably (no jam/double-feed).
- [ ] **Constants.Index** — Tune intake RPM if used; verify positive = toward shooter.
- [ ] **Index** — Verify on robot: positive RPM feeds toward shooter; invert motor in config if backward.
- [ ] **Constants.Tower** — Tune tower shoot RPM; verify positive = toward shooter.
- [ ] **Constants.Tower** — Tune if tower runs during intake.
- [ ] **Tower** — Verify on robot: positive RPM = toward shooter; invert in config if backward.

---

## Intake

- [ ] **Constants.Intake** — On robot: intake fully retracted → read encoder (or zero), set kDeployPositionRetracted.
- [ ] **Constants.Intake** — On robot: intake fully extended → read encoder, set kDeployPositionExtended.
- [ ] **Intake** — Tune wheel PID/kV on robot if wheels slip or overshoot; verify positive = pull in.
- [ ] **Intake** — Verify deploy motor direction: extend() = arm out; flip invert in config if reversed.

---

## Climber

- [ ] **Constants.Climber** — On robot: climber at full climb → read encoder, set kPositionClimbed.
- [ ] **Constants.Climber** — On robot: climber retracted → read encoder (or zero), set kPositionRetracted.
- [ ] **Climber** — Verify follower alignment (Aligned vs Opposed) so both sides move together.
- [ ] **Climber** — On first boot: ensure encoder zero matches retracted position, or set position in Phoenix tuner.

---

## Vision & odometry

- [ ] **Limelight** — Tune vision std devs: smaller = trust vision more; larger = trust wheels more.
- [ ] **AutoAimDriveUsingMegaTagLocations** — Confirm RawHeading convention (0 = +X, 90 = +Y) matches field coords.
- [ ] **AutoAimDriveUsingMegaTagLocations** — Tune rotSpeedMultiplier and baseSpeed on robot for auto-aim.

---

## Locations & field

- [ ] **Locations** — Set hub/shuttle (x, y, z) from official field drawings (meters); z = target center height.
- [ ] **Locations** — Replace HUB_BLUE and HUB_RED placeholder coords with real hub center coords.

---

## Commands & timing

- [ ] **ShootCommand** — Tune feed duration (s) so one ball reliably leaves; too short = no shot, too long = double feed.
- [ ] **ShootCommand** — Adjust prepare timeout if shooter is slow to reach setpoint or to fail faster when something is wrong.

---

## Test commands (bench)

- [ ] **TestCommands** — Adjust flywheel test RPM (e.g. 0.5 of max) if 50% is too fast/slow for bench.
- [ ] **TestCommands** — Reduce climber test voltage (6 V) if too aggressive on bench.
