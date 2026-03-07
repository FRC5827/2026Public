package frc.robot.subsystems.hopper.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs;

    private final LoggedTunableNumber indexerVoltage =
            new LoggedTunableNumber("Hopper/Indexer/Voltage", 4.0);

    public Indexer(IndexerIO io) {
        this.io = io;
        this.inputs = new IndexerIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Hopper/Indexer", inputs);
        Logger.recordOutput(
                "PerformanceMonitor/Hopper/Indexer", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public Command runIndexer() {
        return this.startEnd(
                () -> io.setVoltage(indexerVoltage.getAsDouble()), () -> io.setVoltage(0.0));
    }

    public Command runIndexerReverse() {
        return this.startEnd(
                () -> io.setVoltage(-indexerVoltage.getAsDouble()), () -> io.setVoltage(0.0));
    }

    public void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }
}
