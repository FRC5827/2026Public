package frc.robot.subsystems.intake.deployer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class DeployerIOSim implements DeployerIO {
    private static final double ARM_LENGTH_METERS = 0.560528;
    private static final double ARM_MASS_KG = 6.57;

    private final SingleJointedArmSim deployerSim;

    private final PIDController deployerController;

    private double requestedDeployerVoltage = 0;
    private boolean openLoop = true;
    private double closedLoopPosition = 0;
    private double closedLoopVelocity = 0;
    private double feedforwardVoltage = 0;

    public DeployerIOSim() {
        deployerSim =
                new SingleJointedArmSim(
                        DCMotor.getKrakenX60(1),
                        Deployer.DEPLOYER_GEAR_RATIO,
                        SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG),
                        ARM_LENGTH_METERS,
                        Deployer.DEPLOYER_MIN_ANGLE.in(Radians),
                        Deployer.DEPLOYER_MAX_ANGLE.in(Radians),
                        false,
                        Deployer.DEPLOYER_DEPLOY_ANGLE_RAD.get());
        deployerController =
                new PIDController(Deployer.DEPLOYER_kP.get(), 0, Deployer.DEPLOYER_kD.get());
    }

    @Override
    public void updateInputs(DeployerIOInputs inputs) {
        double appliedVolts;
        if (openLoop) {
            appliedVolts = requestedDeployerVoltage;
        } else {
            appliedVolts =
                    deployerController.calculate(inputs.deployerPositionRadians, closedLoopPosition)
                            + feedforwardVoltage;
        }

        deployerSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        deployerSim.update(0.02);

        inputs.deployerMotorConnected = true;
        inputs.deployerPositionRadians = deployerSim.getAngleRads();
        inputs.deployerVelocityRadPerSec = deployerSim.getVelocityRadPerSec();
        inputs.deployerMotorVoltageVolts = appliedVolts;
        inputs.deployerMotorCurrentAmps = Math.abs(deployerSim.getCurrentDrawAmps());
    }

    @Override
    public void setDeployerVoltage(double voltage) {
        requestedDeployerVoltage = voltage;
        openLoop = true;
    }

    @Override
    public void setDeployerState(
            double targetPosition, double targetVelocity, double feedforwardValue) {
        closedLoopPosition = Units.radiansToRotations(targetPosition);
        closedLoopVelocity = Units.radiansToRotations(targetVelocity);
        feedforwardVoltage = feedforwardValue;
        openLoop = false;
    }

    @Override
    public void setDeployerPosition(double targetPosition) {
        closedLoopPosition = Units.radiansToRotations(targetPosition);
        closedLoopVelocity = 0;
        feedforwardVoltage = 0;
        openLoop = false;
    }

    @Override
    public void updatePID(double kP, double kD) {
        deployerController.setP(kP);
        deployerController.setD(kD);
    }
}
