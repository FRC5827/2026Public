package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim shooterMotorSim;
    private final DCMotor shooterGearbox = DCMotor.getKrakenX60(1);

    private final PIDController shooterController =
            new PIDController(Shooter.shooterKP.get(), 0, Shooter.shooterKD.get());

    private double shooterAppliedVolts = 0.0;
    private boolean closedLoop = false;
    private double shooterSetpoint = 0.0;

    public ShooterIOSim() {
        shooterMotorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                shooterGearbox, 0.001, Shooter.SHOOTER_GEAR_RATIO),
                        shooterGearbox);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        if (closedLoop) {
            shooterAppliedVolts =
                    shooterController.calculate(getShooterVelocityMPS(), shooterSetpoint);
        }

        shooterMotorSim.setInputVoltage(shooterAppliedVolts);
        shooterMotorSim.update(0.02);

        inputs.shooterConnected = true;
        inputs.shooterMotorVoltage = shooterAppliedVolts;
        inputs.shooterMotorVoltageFollower = shooterAppliedVolts;
        inputs.shooterMotorCurrent = Math.abs(shooterMotorSim.getCurrentDrawAmps());
        inputs.shooterMotorCurrentFollower = Math.abs(shooterMotorSim.getCurrentDrawAmps());
        inputs.shooterMotorTemp = 0.0;
        inputs.shooterMotorTempFollower = 0.0;
        inputs.shooterMotorVelocityMPS = getShooterVelocityMPS();
    }

    @Override
    public void setShooterVoltage(double newVoltage) {
        shooterAppliedVolts = newVoltage;
        closedLoop = false;
    }

    @Override
    public void setShooterVelocity(double newVelocity, double feedforwardValue) {
        shooterSetpoint = newVelocity;
        closedLoop = true;
    }

    @Override
    public void updateShooterPID(double kP, double kD) {
        shooterController.setPID(kP, 0, kD);
    }

    private double getShooterVelocityMPS() {
        return shooterMotorSim.getAngularVelocityRadPerSec() * Shooter.SHOOTER_RADIUS_METERS;
    }
}
