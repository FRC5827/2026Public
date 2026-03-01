package frc.robot.subsystems.intake.flywheel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs;
    private final LoggedTunableNumber flywheelVoltage =
            new LoggedTunableNumber("Intake/Flywheel/flywheelVoltage", 8.0);

    public Flywheel(FlywheelIO io) {
        this.io = io;
        this.inputs = new FlywheelIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Flywheel", inputs);
        Logger.recordOutput(
                "PerformanceMonitor/Intake/Flywheel",
                (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public Command runIntake() {
        return this.startEnd(
                () -> io.setFlywheelVoltage(flywheelVoltage.getAsDouble()),
                () -> io.setFlywheelVoltage((0.0)));
    }

    public Command runReverse() {
        return this.startEnd(
                () -> io.setFlywheelVoltage(-flywheelVoltage.getAsDouble()),
                () -> io.setFlywheelVoltage(0.0));
    }

    public void setBrakeMode(boolean brake) {
        io.setFlywheelBrakeMode(brake);
    }
}
