// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final boolean tuningMode = false;
    public static final boolean characterizationMode = false;
    public static final boolean fusionAutosEnabled = false;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final boolean simWithVision = true;

    public static final double ROBOT_LENGTH = 0.8255; // meters (bumper-to-bumper length)

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final int shooterMotorCanbus_ID = 16;
    public static final int shooterMotorFollowerCanbus_ID = 17;
    public static final int turretPitchServo1PWM_ID = 0;
    public static final int turretPitchServo2PWM_ID = 1;
    public static final int turretYawMotorCanbus_ID = 18;
    public static final int turretYawLimitSwitchDIO = 1;

    public static final int hopperIndexerMotorOpenCanbus_ID = 20;
    public static final int hopperKickerMotorOpenCanbus_ID = 19;

    public static final int intakeDeployerMotorCanbus_ID = 22;
    public static final int intakeDeployerCancoder_ID = 23;
    public static final int intakeFlywheelMotorCanbus_ID = 21;
}
