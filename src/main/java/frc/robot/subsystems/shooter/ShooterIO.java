package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean flywheelConnected = false;
        public boolean yawConnected = false;

        public double flywheelMotorVoltage = 0.0;
        public double flywheelMotorVoltageFollower = 0.0;
        public double flywheelMotorCurrent = 0.0;
        public double flywheelMotorCurrentFollower = 0.0;
        public double flywheelMotorTemp = 0.0;
        public double flywheelMotorTempFollower = 0.0;
        public double flywheelMotorVelocityMPS = 0.0;

        public double pitchServoRequestedPosition = 0.0;

        public double yawMotorVoltage = 0.0;
        public double yawMotorCurrent = 0.0;
        public double yawMotorTemperature = 0.0;
        public double yawTurretPositionRotations = 0.0;
        public boolean yawLimitSwitchPressed = false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setFlywheelVoltage(double newVoltage) {}

    public default void setFlyWheelVelocity(double newVelocity, double feedforwardValue) {}

    public default void setPitchAngle(double newAngle) {}

    public default void setYawVoltage(double newVoltage) {}

    public default void setYawPosition(double newPosition) {}

    public default void setYawState(
            double newPosition, double newVelocity, double feedforwardValue) {}

    public default void updateFlywheelPID(double kP, double kD) {}

    public default void updateYawPID(double kP, double kD) {}

    public default void updateYawLimits(double minRotations, double maxRotations) {}

    public default void zeroYaw() {}
}
