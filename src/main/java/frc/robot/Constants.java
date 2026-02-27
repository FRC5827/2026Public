// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final boolean tuningMode = true;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final boolean simWithVision = false;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final CANBus shooterCANBus = new CANBus("canivore");
    public static final int shooterFlywheelMotorCanbus_ID = 16;
    public static final int shooterFlywheelMotorFollowerCanbus_ID = 17;
    public static final int shooterPitchServo1PWM_ID = 0;
    public static final int shooterPitchServo2PWM_ID = 1;
    public static final int shooterYawMotorCanbus_ID = 18;
    public static final int shooterYawLimitSwitchDIO = 0;

    public static final int hopperIndexerMotorOpenCanbus_ID = 20; // need to change
    public static final int hopperKickerMotorOpenCanbus_ID = 21; // need to change

    public static final int intakeDeployerMotorCanbus_ID = 22;
    public static final int intakeDeployerCancoder_ID = 23;
}
