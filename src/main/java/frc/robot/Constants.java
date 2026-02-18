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

    public static int shooterFiringMotorOpenCanbus_ID = 16;
    public static int shooterFiringMotorFollowerOpenCanbus_ID = 17;
    public static int shooterYawMotorCanbus_ID = 18;
    public static int yawLimitSwitchDIO = 0;
}
