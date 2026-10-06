# CLAUDE.md

## Project Overview

FRC 5827 robot code for the 2026 season. Java-based, built on WPILib with AdvantageKit logging and CTRE Phoenix 6 hardware.

## Building

JAVA_HOME is not on PATH. You must set it when running Gradle:

```bash
JAVA_HOME="C:/Users/Public/wpilib/2026/jdk" ./gradlew build
```

This compiles, runs spotless formatting, and checks for errors. Always run this after making changes.

Spotless auto-formatting runs as part of the build (`compileJava` depends on `spotlessApply`). It uses Google Java Format (AOSP style), 4-space indentation, and removes unused imports. Do not fight the formatter — just run the build and it will fix formatting for you.

## Deploying to the Robot

```bash
JAVA_HOME="C:/Users/Public/wpilib/2026/jdk" ./gradlew deploy
```

Requires network connection to the roboRIO (IP 10.93.17.2 or USB at 172.22.11.2).

## Project Structure

- `src/main/java/frc/robot/` — all robot code
  - `RobotContainer.java` — subsystem instantiation, button bindings, auto chooser
  - `Constants.java` — CAN IDs, DIO ports, mode switching (REAL/SIM)
  - `FieldConstants.java` — field geometry, AprilTag layout, physical constants (e.g. gravity)
  - `subsystems/` — each subsystem follows the IO-layer pattern (see below)
  - `commands/` — standalone commands (e.g. DriveCommands)
  - `util/` — utilities (LoggedTunableNumber, AllianceFlipUtil, PhoenixUtil)
- `tools/` — Python scripts for log management (see below)
- `logs/` — downloaded `.wpilog` files (gitignored)
- `vendordeps/` — third-party library JSON configs

## IO-Layer Pattern

Every subsystem uses a hardware abstraction layer:
- `SubsystemIO.java` — interface defining hardware methods and an `Inputs` inner class
- `SubsystemIOReal.java` — real hardware implementation (TalonFX, servos, sensors)
- `SubsystemIOSim.java` — simulation implementation
- `Subsystem.java` — subsystem logic, takes an `IO` instance in its constructor

`RobotContainer` picks the IO implementation based on `Constants.currentMode` (REAL, SIM, or default/replay). When adding new hardware interactions, add them to the IO interface first, then implement in Real/Sim.

## Coding Conventions

- Use `LoggedTunableNumber` for any value that should be adjustable at runtime via NetworkTables. This is preferred over hardcoded constants for gains, setpoints, and thresholds.
- Use WPILib geometry types (`Rotation2d`, `Translation2d`, `Pose2d`, etc.) and their built-in methods for angle math, normalization, and transforms. Do not reimplement angle wrapping or bearing math with raw doubles and `Math.atan2` — use `Rotation2d.minus()`, `.getAngle()`, `.getRotations()`, etc.
- Use `MathUtil.clamp()` instead of `Math.max(min, Math.min(max, value))`.
- Physical constants (gravity, field dimensions) belong in `FieldConstants`, not as local constants in subsystems.
- Motor PID gains should be tunable via `LoggedTunableNumber` and applied in `checkForPIDUpdates()` so they can be changed at runtime. They should be read from the tunable when constructing motor configs (e.g. `yawConfig.Slot0.kP = Shooter.yawMotorP.getAsDouble()`), not hardcoded in the IO layer.
- Hardware soft limits (e.g. TalonFX `SoftwareLimitSwitch`) should be configured on the motor controller where possible, rather than only enforced in software.
- Tunable number names that reference units should include the unit in the name (e.g. `targetFlywheelRadiansPerSecond`, not `targetFlywheelRPS` when the value is in rad/s).
- The `static final` visibility of tunables that are accessed from the IO layer should be package-private (`static final`, no access modifier), not `private static final`.

## Tools

### `tools/pull_logs.py`
Downloads `.wpilog` files from the roboRIO USB stick over SSH. Automatically matches them to Driver Station `.dslog` files by timestamp. Manages USB space by deleting old already-downloaded logs.

```bash
python tools/pull_logs.py              # download new logs
python tools/pull_logs.py --list       # list logs on roboRIO
python tools/pull_logs.py --all        # re-download everything
python tools/pull_logs.py --no-cleanup # skip USB cleanup
```

Requires SSH access to the roboRIO (team 5827). Searches multiple addresses automatically.

### `tools/parse_dslog.py`
Parses binary Driver Station `.dslog` v4 files and reports overrun analysis, CPU/CAN stats, trip time distributions, and worst-case timing.

```bash
python tools/parse_dslog.py <path_to_dslog_file>
python tools/parse_dslog.py   # auto-detects first .dslog in logs/latest/
```

### `tools/parse_dsevents.py`
Parses binary Driver Station `.dsevents` files and reports the startup sequence, loop overrun breakdowns (per-subsystem timing from WPILib's Tracer), and errors/warnings with deduplication.

```bash
python tools/parse_dsevents.py <path_to_dsevents_file>
python tools/parse_dsevents.py            # auto-detects in logs/latest/
python tools/parse_dsevents.py --verbose  # also print full event timeline
```

### `logs/latest/`
`pull_logs.py` automatically maintains a `logs/latest/` directory containing the most recent log set (`.wpilog` + matching `.dslog` + `.dsevents`). It is refreshed only when new logs are downloaded, so the contents always represent the last pull from the robot. The parsing tools auto-detect files here when no path is given.
