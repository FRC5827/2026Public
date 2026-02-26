package frc.robot.subsystems.hopper.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {
    private final FlywheelSim sim;

    private final DCMotor motor = DCMotor.getKrakenX60(1);

    private double requestedVoltage = 0;

    public IndexerIOSim() {
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, 0.1, 1), motor);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        sim.setInputVoltage(MathUtil.clamp(requestedVoltage, -12, 12));
        sim.update(0.02);
        inputs.connected = true;
        inputs.angularVelRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.inputVoltage = sim.getInputVoltage();
    }

    @Override
    public void setVoltage(double voltage) {
        requestedVoltage = voltage;
    }
}
