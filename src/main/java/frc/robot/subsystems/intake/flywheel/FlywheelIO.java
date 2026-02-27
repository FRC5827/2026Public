package frc.robot.subsystems.intake.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs {
        public boolean connected = false;
        public double angularAccelRadPerSecSq = 0;
        public double angularVelRadPerSec = 0;
        public double currentAmps = 0.0;
        public double inputVoltage = 0.0;
        public double motorTemperatureCelsius = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(FlywheelIOInputs inputs) {}

    /** Run the flywheel motor at the specified voltage value */
    public default void setFlywheelVoltage(double voltage) {}

    public default double getFlywheelVelocity(FlywheelIOInputs inputs) {
        return 0.0;
    }

    public default void setFlywheelBrakeMode(boolean brake) {}
}
