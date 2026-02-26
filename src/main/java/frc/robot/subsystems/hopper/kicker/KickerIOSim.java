package frc.robot.subsystems.hopper.kicker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Kicker IO implementation for simulation. Simulates left and right kicker motors where left
 * follows right with inverted direction.
 */
public class KickerIOSim implements KickerIO {
    private final FlywheelSim sim;

    private final DCMotor motor = DCMotor.getKrakenX44(1);

    private double requestedVoltage = 0;

    public KickerIOSim() {
        sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, 0.1, 1), motor);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
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
