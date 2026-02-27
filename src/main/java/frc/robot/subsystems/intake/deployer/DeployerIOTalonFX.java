package frc.robot.subsystems.intake.deployer;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class DeployerIOTalonFX implements DeployerIO {
    private static final int CURRENT_LIMIT = 40;

    private final TalonFX deployerMotor;
    private final TalonFXConfiguration deployerConfig;

    private final PositionVoltage positionRequest;

    private final StatusSignal<Double> deployerTargetSignal;
    private final StatusSignal<Double> deployerErrorSignal;
    private final StatusSignal<Angle> deployerPositionSignal;
    private final StatusSignal<AngularVelocity> deployerVelocitySignal;
    private final StatusSignal<Voltage> deployerMotorVoltageSignal;
    private final StatusSignal<Current> deployerMotorCurrentSignal;
    private final StatusSignal<Temperature> deployerMotorTempSignal;

    public DeployerIOTalonFX() {
        deployerMotor = new TalonFX(Constants.intakeDeployerMotorCanbus_ID);
        // default to up
        positionRequest = new PositionVoltage(Deployer.DEPLOYER_RETRACT_ANGLE_RAD.get());
        deployerConfig = new TalonFXConfiguration();
        deployerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deployerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        deployerConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        deployerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        deployerConfig.Feedback.FeedbackRemoteSensorID = Constants.intakeDeployerCancoder_ID;
        deployerConfig.Feedback.SensorToMechanismRatio = Deployer.DEPLOYER_GEAR_RATIO;
        deployerConfig.Voltage.PeakForwardVoltage = 3.0;
        deployerConfig.Voltage.PeakReverseVoltage = -3.0;
        deployerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        deployerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        deployerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Deployer.DEPLOYER_MAX_ANGLE.in(Rotations);
        deployerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Deployer.DEPLOYER_MIN_ANGLE.in(Rotations);

        deployerConfig.Slot0.kP = Deployer.DEPLOYER_kP.get();
        deployerConfig.Slot0.kD = Deployer.DEPLOYER_kD.get();
        PhoenixUtil.tryUntilOk(
                5, () -> deployerMotor.getConfigurator().apply(deployerConfig, 0.25));
        deployerMotor.setVoltage(0);

        deployerTargetSignal = deployerMotor.getClosedLoopReference();
        deployerErrorSignal = deployerMotor.getClosedLoopError();
        deployerPositionSignal = deployerMotor.getPosition();
        deployerVelocitySignal = deployerMotor.getVelocity();
        deployerMotorVoltageSignal = deployerMotor.getMotorVoltage();
        deployerMotorCurrentSignal = deployerMotor.getSupplyCurrent();
        deployerMotorTempSignal = deployerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                deployerTargetSignal,
                deployerErrorSignal,
                deployerPositionSignal,
                deployerVelocitySignal,
                deployerMotorVoltageSignal,
                deployerMotorCurrentSignal,
                deployerMotorTempSignal);
        ParentDevice.optimizeBusUtilizationForAll(deployerMotor);
    }

    @Override
    public void updateInputs(DeployerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                deployerTargetSignal,
                deployerErrorSignal,
                deployerPositionSignal,
                deployerVelocitySignal,
                deployerMotorVoltageSignal,
                deployerMotorCurrentSignal,
                deployerMotorTempSignal);

        inputs.deployerMotorConnected = deployerMotor.isConnected();
        inputs.deployerPositionRadians =
                Units.rotationsToRadians(deployerPositionSignal.getValueAsDouble());
        inputs.deployerVelocityRadPerSec =
                Units.rotationsToRadians(deployerVelocitySignal.getValueAsDouble());
        inputs.deployerMotorVoltageVolts = deployerMotorVoltageSignal.getValueAsDouble();
        inputs.deployerMotorCurrentAmps = deployerMotorCurrentSignal.getValueAsDouble();
        inputs.deployerMotorTempCelsius = deployerMotorTempSignal.getValueAsDouble();
    }

    @Override
    public void setDeployerBrakeMode(boolean brake) {
        deployerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setDeployerVoltage(double voltage) {
        deployerMotor.setVoltage(voltage);
    }

    @Override
    public void setDeployerState(
            double targetPosition, double targetVelocity, double feedforwardValue) {
        deployerMotor.setControl(
                positionRequest
                        .withPosition(targetPosition)
                        .withVelocity(targetVelocity)
                        .withFeedForward(feedforwardValue));
    }

    @Override
    public void setDeployerPosition(double targetPosition) {
        deployerMotor.setControl(
                positionRequest.withPosition(targetPosition).withVelocity(0).withFeedForward(0));
    }

    @Override
    public void updatePID(double kP, double kD) {
        deployerConfig.Slot0.kP = kP;
        deployerConfig.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(
                5, () -> deployerMotor.getConfigurator().apply(deployerConfig.Slot0, 0.25));
    }
}
