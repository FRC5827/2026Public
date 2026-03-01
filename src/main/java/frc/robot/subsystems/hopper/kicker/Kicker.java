package frc.robot.subsystems.hopper.kicker;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs;

    private final LoggedTunableNumber kickerVoltage =
            new LoggedTunableNumber("Hopper/Kicker/Voltage", 4.0);

    public Kicker(KickerIO io) {
        this.io = io;
        this.inputs = new KickerIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Hopper/Kicker", inputs);
        Logger.recordOutput(
                "PerformanceMonitor/Hopper/Kicker", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public Command runKicker() {
        return this.startEnd(
                () -> io.setVoltage(kickerVoltage.getAsDouble()), () -> io.setVoltage(0.0));
    }

    public Command runKickerReverse() {
        return this.startEnd(
                () -> io.setVoltage(-kickerVoltage.getAsDouble()), () -> io.setVoltage(0.0));
    }

    public void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }
}
