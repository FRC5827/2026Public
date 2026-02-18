package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

    private final DCMotorSim firingMotorSim;
    private final DCMotorSim firingMotorFollowerSim;
    private final DCMotorSim yawControlMotorSim;

    private final DCMotor firingMotor = DCMotor.getKrakenX60(1);
    private final DCMotor yawControlMotor = DCMotor.getKrakenX60(1);

    private final double maxVoltage = 12.0;

    private final PIDController yawController = new PIDController(.5, 0, 0);
    private final PIDController firingController = new PIDController(.5, 0, 0);

    private double requestedYawControlVoltage = 0.0;
    private double requestedPitchControlVoltage = 0.0;

    private boolean openLoop = false;
    private double actuatorPosition = 0.5;

    public ShooterIOSim() {
        firingMotorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(firingMotor, 0.001, 1), firingMotor);

        firingMotorFollowerSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(firingMotor, 0.001, 1), firingMotor);

        yawControlMotorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(yawControlMotor, 0.001, 1),
                        yawControlMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        firingMotorSim.update(0.02);
        firingMotorFollowerSim.update(0.02);
        yawControlMotorSim.update(0.02);

        inputs.shooterConnected = true;

        inputs.shooterPositionYaw = yawControlMotorSim.getAngularPositionRad();

        inputs.firingAngularVelocityRPS = firingMotorSim.getAngularVelocityRadPerSec();
        inputs.firingMotorCurrentAmps = firingMotorSim.getCurrentDrawAmps();
        inputs.firingMotorFollowerCurrentAmps = firingMotorFollowerSim.getCurrentDrawAmps();
        inputs.yawControlMotorCurrentAmps = yawControlMotorSim.getCurrentDrawAmps();

        if (openLoop) {
            inputs.yawControlMotorAppliedVolts = requestedYawControlVoltage;
            inputs.pitchControlMotorAppliedVolts = requestedPitchControlVoltage;

            inputs.firingMotorAppliedVolts = firingMotorSim.getInputVoltage();
            inputs.firingMotorFollowerAppliedVolts = firingMotorFollowerSim.getInputVoltage();
        } else {
            inputs.yawControlMotorAppliedVolts =
                    MathUtil.clamp(
                            yawController.calculate(yawControlMotorSim.getAngularPositionRad()),
                            -maxVoltage,
                            maxVoltage);

            inputs.firingMotorAppliedVolts =
                    MathUtil.clamp(
                            firingController.calculate(
                                    firingMotorSim.getAngularVelocityRadPerSec()),
                            -maxVoltage,
                            maxVoltage);
            System.out.println(
                    firingController.getSetpoint()
                            + " "
                            + firingMotorSim.getAngularVelocityRadPerSec());

            inputs.firingMotorFollowerAppliedVolts = inputs.firingMotorAppliedVolts * -1;
            firingMotorSim.setInputVoltage(inputs.firingMotorAppliedVolts);
            firingMotorFollowerSim.setInputVoltage(inputs.firingMotorFollowerAppliedVolts);
        }
        yawControlMotorSim.setInputVoltage(inputs.yawControlMotorAppliedVolts);

        inputs.actuatorPosition = actuatorPosition;
    }

    public void setTurretPitchMotorVoltage(double voltage) {
        requestedYawControlVoltage = voltage;
    }

    public void setTurretYawMotorVoltage(double voltage) {
        yawControlMotorSim.setInputVoltage(voltage);
    }

    public void setTurretPitchPosition(double newPosition) {}

    public void setTurretYawPosition(double newPosition) {
        yawController.setSetpoint(newPosition);
    }

    public void setTurretFiringVoltage(double newVoltage) {
        firingMotorSim.setInputVoltage(newVoltage);
        firingMotorFollowerSim.setInputVoltage(newVoltage * -1);
    }

    public void setTurretTargetFiringVelocity(double newVelocity) {
        firingController.setSetpoint(newVelocity);
    }

    public void updatePIDFiringMotors(double kP, double kD) {
        firingController.setPID(kP, 0, kD);
    }

    public void updatePIDYawMotor(double kP, double kD) {
        yawController.setPID(kP, 0, kD);
    }

    @Override
    public void setActuatorPosition(double position) {
        actuatorPosition = position;
    }
}
