package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean shooterConnected = false;

        public double shooterMotorVoltage = 0.0;
        public double shooterMotorVoltageFollower = 0.0;
        public double shooterMotorCurrent = 0.0;
        public double shooterMotorCurrentFollower = 0.0;
        public double shooterMotorTemp = 0.0;
        public double shooterMotorTempFollower = 0.0;
        public double shooterMotorVelocityMPS = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setShooterVoltage(double newVoltage) {}

    public default void setShooterVelocity(double newVelocity, double feedforwardValue) {}

    public default void updateShooterPID(double kP, double kD) {}
}
