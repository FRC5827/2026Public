package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ShooterIOReal implements ShooterIO {
    private static final double CURRENT_LIMIT = 40.0;

    private final TalonFXConfiguration shooterConfig;
    private final TalonFX shooterMotor, shooterMotorFollower;

    private final StatusSignal<Voltage> shooterMotorVoltage;
    private final StatusSignal<Voltage> shooterMotorVoltageFollower;
    private final StatusSignal<Current> shooterMotorCurrent;
    private final StatusSignal<Current> shooterMotorCurrentFollower;
    private final StatusSignal<Temperature> shooterMotorTemp;
    private final StatusSignal<Temperature> shooterMotorTempFollower;
    private final StatusSignal<AngularVelocity> shooterMotorVelocity;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public ShooterIOReal() {
        shooterMotor = new TalonFX(Constants.shooterMotorCanbus_ID);
        shooterMotorFollower = new TalonFX(Constants.shooterMotorFollowerCanbus_ID);
        shooterMotorFollower.setControl(
                new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        shooterConfig.Feedback.SensorToMechanismRatio = Shooter.SHOOTER_GEAR_RATIO;
        shooterConfig.Slot0.kP = Shooter.shooterKP.get();
        shooterConfig.Slot0.kD = Shooter.shooterKD.get();

        PhoenixUtil.tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(shooterConfig, 0.25));
        PhoenixUtil.tryUntilOk(
                5, () -> shooterMotorFollower.getConfigurator().apply(shooterConfig, 0.25));

        shooterMotorVoltage = shooterMotor.getMotorVoltage();
        shooterMotorVoltageFollower = shooterMotorFollower.getMotorVoltage();
        shooterMotorCurrent = shooterMotor.getSupplyCurrent();
        shooterMotorCurrentFollower = shooterMotorFollower.getSupplyCurrent();
        shooterMotorTemp = shooterMotor.getDeviceTemp();
        shooterMotorTempFollower = shooterMotorFollower.getDeviceTemp();
        shooterMotorVelocity = shooterMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                shooterMotorVoltage,
                shooterMotorVoltageFollower,
                shooterMotorCurrent,
                shooterMotorCurrentFollower,
                shooterMotorTemp,
                shooterMotorTempFollower,
                shooterMotorVelocity);

        ParentDevice.optimizeBusUtilizationForAll(shooterMotor, shooterMotorFollower);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var shooterStatus =
                BaseStatusSignal.refreshAll(
                        shooterMotorVoltage,
                        shooterMotorVoltageFollower,
                        shooterMotorCurrent,
                        shooterMotorCurrentFollower,
                        shooterMotorTemp,
                        shooterMotorTempFollower,
                        shooterMotorVelocity);

        inputs.shooterConnected = shooterStatus.isOK();
        inputs.shooterMotorVoltage = shooterMotorVoltage.getValueAsDouble();
        inputs.shooterMotorVoltageFollower = shooterMotorVoltageFollower.getValueAsDouble();
        inputs.shooterMotorCurrent = shooterMotorCurrent.getValueAsDouble();
        inputs.shooterMotorCurrentFollower = shooterMotorCurrentFollower.getValueAsDouble();
        inputs.shooterMotorTemp = shooterMotorTemp.getValueAsDouble();
        inputs.shooterMotorTempFollower = shooterMotorTempFollower.getValueAsDouble();
        inputs.shooterMotorVelocityMPS =
                Units.rotationsToRadians(shooterMotorVelocity.getValueAsDouble())
                        * Shooter.SHOOTER_RADIUS_METERS;
    }

    @Override
    public void setShooterVoltage(double newVoltage) {
        shooterMotor.setControl(voltageRequest.withOutput(newVoltage));
    }

    @Override
    public void setShooterVelocity(double newVelocity, double feedforwardValue) {
        double velocityRotationsPerSecond =
                Units.radiansToRotations(newVelocity / Shooter.SHOOTER_RADIUS_METERS);
        shooterMotor.setControl(
                velocityRequest
                        .withVelocity(velocityRotationsPerSecond)
                        .withFeedForward(feedforwardValue));
    }

    @Override
    public void updateShooterPID(double kP, double kD) {
        shooterConfig.Slot0.kP = kP;
        shooterConfig.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(shooterConfig, 0.25));
        PhoenixUtil.tryUntilOk(
                5, () -> shooterMotorFollower.getConfigurator().apply(shooterConfig, 0.25));
    }
}
