package frc.robot.subsystems.hopper.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        public boolean connected = false;
        public double angularVelRadPerSec = 0;
        public double currentAmps = 0.0;
        public double inputVoltage = 0.0;
        public double motorTemperatureCelsius = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(IndexerIOInputs inputs) {}

    /** Run the indexer motor at the specified voltage value */
    public default void setVoltage(double voltage) {}

    /** Set brake mode */
    public default void setBrakeMode(boolean brake) {}
}
