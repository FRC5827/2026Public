package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public boolean yawConnected = false;

        public double pitchServoRequestedPosition = 0.0;

        public double yawMotorVoltage = 0.0;
        public double yawMotorCurrent = 0.0;
        public double yawMotorTemperature = 0.0;
        public double yawTurretPositionRotations = 0.0;

        public boolean yawLimitSwitchPressed = false;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setPitchAngle(double newAngle) {}

    public default void setYawVoltage(double newVoltage) {}

    public default void setYawPosition(double newPosition) {}

    public default void setYawState(
            double newPosition, double newVelocity, double feedforwardValue) {}

    public default void updateYawPID(double kP, double kD) {}

    public default void updateYawLimits(double minRotations, double maxRotations) {}

    public default void zeroYaw(double zeroingOffset) {}

    public default void setBrakeMode(boolean brake) {}
}
