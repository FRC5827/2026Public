package frc.robot.subsystems.intake.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
    private final FlywheelSim flywheelSim;
    private final DCMotor flywheelMotor = DCMotor.getKrakenX60(1);
    private double flywheelRequestedVoltage = 0;

    public FlywheelIOSim() {
        flywheelSim =
                new FlywheelSim(
                        LinearSystemId.createFlywheelSystem(flywheelMotor, 0.01, 1), flywheelMotor);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        flywheelSim.setInputVoltage(MathUtil.clamp(flywheelRequestedVoltage, -12, 12));
        flywheelSim.update(0.02);
        inputs.connected = true;
        inputs.angularAccelRadPerSecSq = flywheelSim.getAngularAccelerationRadPerSecSq();
        inputs.angularVelRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
        inputs.currentAmps = flywheelSim.getCurrentDrawAmps();
        inputs.inputVoltage = flywheelSim.getInputVoltage();
    }

    @Override
    public void setFlywheelVoltage(double voltage) {
        flywheelRequestedVoltage = voltage;
    }

    @Override
    public double getFlywheelVelocity(FlywheelIOInputs inputs) {
        return inputs.angularVelRadPerSec;
    }
}
