package frc.robot.subsystems.hopper.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {

    @AutoLog
    public static class KickerIOInputs {
        public boolean connected = false;
        public double angularVelRadPerSec = 0;
        public double currentAmps = 0.0;
        public double inputVoltage = 0.0;
        public double motorTemperatureCelsius = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(KickerIOInputs inputs) {}

    /** Run the kicker motor at the specified voltage value */
    public default void setVoltage(double voltage) {}

    /** Set brake mode */
    public default void setBrakeMode(boolean brake) {}
}
