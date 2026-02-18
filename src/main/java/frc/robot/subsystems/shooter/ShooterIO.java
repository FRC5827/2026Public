package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean shooterConnected = false;

        // Pitch is above the horizon, Yaw is rotating the horizon.
        public double shooterPositionYaw = 0.0;
        public double shooterPositionPitch = 0.0;
        public double firingAngularVelocityRPS = 0.0;

        public double yawControlMotorAppliedVolts = 0.0;
        public double pitchControlMotorAppliedVolts = 0.0;
        public double firingMotorAppliedVolts = 0.0;
        public double firingMotorFollowerAppliedVolts = 0.0;

        public double yawControlMotorCurrentAmps = 0.0;
        public double pitchControlMotorCurrentAmps = 0.0;
        public double firingMotorCurrentAmps = 0.0;
        public double firingMotorFollowerCurrentAmps = 0.0;

        public double yawControlMotorTemperature = 0.0;
        public double pitchControlMotorTemperature = 0.0;
        public double firingMotorTemperature = 0.0;
        public double firingMotorFollowerTemperature = 0.0;

        public double actuatorPosition = 0.0;
        public boolean yawLimitSwitchPressed = false;
        public boolean yawEncoderZeroed = false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setTurretPitchPosition(double newPosition) {}

    public default void setTurretYawPosition(double newPosition) {}

    public default void setTurretFiringVoltage(double newVelocity) {}

    public default void setTurretTargetFiringVelocity(double newVelocity) {}

    public default void setTurretPitchMotorVoltage(double newVoltage) {}

    public default void setTurretYawMotorVoltage(double newVoltage) {}

    public default void updatePIDFiringMotors(double kP, double kD) {}

    public default void updatePIDPitchMotor(double kP, double kD) {}

    public default void updatePIDYawMotor(double kP, double kD) {}

    public default void setActuatorPosition(double position) {}
}
