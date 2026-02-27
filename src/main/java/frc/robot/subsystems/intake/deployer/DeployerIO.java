package frc.robot.subsystems.intake.deployer;

import org.littletonrobotics.junction.AutoLog;

public interface DeployerIO {

    @AutoLog
    public static class DeployerIOInputs {
        public boolean deployerMotorConnected = false;
        public double deployerPositionRadians = 0.0;
        public double deployerVelocityRadPerSec = 0;
        // Measured voltage, not reuested voltage
        public double deployerMotorVoltageVolts = 0.0;
        // Measured current, not requested current
        public double deployerMotorCurrentAmps = 0.0;
        public double deployerMotorTempCelsius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(DeployerIOInputs inputs) {}

    /** Requests the pivot to move to a target position (in rotations) */
    public default void setDeployerPosition(double position) {}

    public default void setDeployerState(
            double targetPosition, double targetVelocity, double feedforwardValue) {}

    public default void setDeployerVoltage(double voltage) {}

    public default void setDeployerBrakeMode(boolean brake) {}

    public default void updatePID(double kP, double kD) {}
}
