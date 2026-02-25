package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ShooterIOReal implements ShooterIO {
    private static final double CURRENT_LIMIT = 40.0;

    private final TalonFXConfiguration flywheelConfig;
    private final TalonFX flywheelMotor, flywheelMotorFollower;
    private final Servo pitchServo1, pitchServo2;

    private final TalonFXConfiguration yawConfig;
    private final TalonFX yawMotor;
    private final DigitalInput yawLimitSwitch;

    private final StatusSignal<Voltage> flywheelMotorVoltage;
    private final StatusSignal<Voltage> flywheelMotorVoltageFollower;
    private final StatusSignal<Current> flywheelMotorCurrent;
    private final StatusSignal<Current> flywheelMotorCurrentFollower;
    private final StatusSignal<Temperature> flywheelMotorTemp;
    private final StatusSignal<Temperature> flywheelMotorTempFollower;
    private final StatusSignal<AngularVelocity> flywheelMotorVelocity;

    private final StatusSignal<Voltage> yawMotorVoltage;
    private final StatusSignal<Current> yawMotorCurrent;
    private final StatusSignal<Temperature> yawMotorTemperature;
    private final StatusSignal<Angle> yawMotorPosition;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public ShooterIOReal() {
        flywheelMotor =
                new TalonFX(Constants.shooterFlywheelMotorCanbus_ID, Constants.shooterCANBus);
        flywheelMotorFollower =
                new TalonFX(
                        Constants.shooterFlywheelMotorFollowerCanbus_ID, Constants.shooterCANBus);
        flywheelMotorFollower.setControl(
                new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        flywheelConfig.Feedback.SensorToMechanismRatio = Shooter.FLYWHEEL_GEAR_RATIO;
        flywheelConfig.Slot0.kP = Shooter.flywheelKP.get();
        flywheelConfig.Slot0.kD = Shooter.flywheelKD.get();

        PhoenixUtil.tryUntilOk(
                5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));
        PhoenixUtil.tryUntilOk(
                5, () -> flywheelMotorFollower.getConfigurator().apply(flywheelConfig, 0.25));

        pitchServo1 = new Servo(Constants.shooterPitchServo1PWM_ID);
        pitchServo2 = new Servo(Constants.shooterPitchServo2PWM_ID);
        pitchServo1.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        pitchServo2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        yawMotor = new TalonFX(Constants.shooterYawMotorCanbus_ID, Constants.shooterCANBus);
        yawLimitSwitch = new DigitalInput(Constants.shooterYawLimitSwitchDIO);

        yawConfig = new TalonFXConfiguration();
        yawConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        yawConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        yawConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        yawConfig.Feedback.SensorToMechanismRatio = Shooter.YAW_GEAR_RATIO;
        yawConfig.Slot0.kP = Shooter.yawKP.getAsDouble();
        yawConfig.Slot0.kD = Shooter.yawKD.getAsDouble();
        yawConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        yawConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Shooter.yawMinRotations.get();
        yawConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        yawConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Shooter.yawMaxRotations.get();
        PhoenixUtil.tryUntilOk(5, () -> yawMotor.getConfigurator().apply(yawConfig, 0.25));

        flywheelMotorVoltage = flywheelMotor.getMotorVoltage();
        flywheelMotorVoltageFollower = flywheelMotorFollower.getMotorVoltage();
        flywheelMotorCurrent = flywheelMotor.getSupplyCurrent();
        flywheelMotorCurrentFollower = flywheelMotorFollower.getSupplyCurrent();
        flywheelMotorTemp = flywheelMotor.getDeviceTemp();
        flywheelMotorTempFollower = flywheelMotorFollower.getDeviceTemp();
        flywheelMotorVelocity = flywheelMotor.getVelocity();

        yawMotorVoltage = yawMotor.getMotorVoltage();
        yawMotorCurrent = yawMotor.getSupplyCurrent();
        yawMotorTemperature = yawMotor.getDeviceTemp();
        yawMotorPosition = yawMotor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                flywheelMotorVoltage,
                flywheelMotorVoltageFollower,
                flywheelMotorCurrent,
                flywheelMotorCurrentFollower,
                flywheelMotorTemp,
                flywheelMotorTempFollower,
                flywheelMotorVelocity,
                yawMotorVoltage,
                yawMotorCurrent,
                yawMotorTemperature,
                yawMotorPosition);

        ParentDevice.optimizeBusUtilizationForAll(flywheelMotor, flywheelMotorFollower, yawMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var flywheelStatus =
                BaseStatusSignal.refreshAll(
                        flywheelMotorVoltage,
                        flywheelMotorVoltageFollower,
                        flywheelMotorCurrent,
                        flywheelMotorCurrentFollower,
                        flywheelMotorTemp,
                        flywheelMotorTempFollower,
                        flywheelMotorVelocity);

        inputs.flywheelConnected = flywheelStatus.isOK();
        inputs.flywheelMotorVoltage = flywheelMotorVoltage.getValueAsDouble();
        inputs.flywheelMotorVoltageFollower = flywheelMotorVoltageFollower.getValueAsDouble();
        inputs.flywheelMotorCurrent = flywheelMotorCurrent.getValueAsDouble();
        inputs.flywheelMotorCurrentFollower = flywheelMotorCurrentFollower.getValueAsDouble();
        inputs.flywheelMotorTemp = flywheelMotorTemp.getValueAsDouble();
        inputs.flywheelMotorTempFollower = flywheelMotorTempFollower.getValueAsDouble();
        inputs.flywheelMotorVelocityMPS =
                flywheelMotorVelocity.getValue().in(RadiansPerSecond)
                        * Shooter.FLYWHEEL_RADIUS_METERS
                        / 2;
        // divide by 2 because only 1 side of ball is controlled by motor

        inputs.pitchServoRequestedPosition = pitchServo1.get();

        var yawStatus =
                BaseStatusSignal.refreshAll(
                        yawMotorVoltage, yawMotorCurrent, yawMotorTemperature, yawMotorPosition);

        inputs.yawConnected = yawStatus.isOK();
        inputs.yawMotorVoltage = yawMotorVoltage.getValueAsDouble();
        inputs.yawMotorCurrent = yawMotorCurrent.getValueAsDouble();
        inputs.yawMotorTemperature = yawMotorTemperature.getValueAsDouble();
        inputs.yawTurretPositionRotations = yawMotorPosition.getValueAsDouble();
        inputs.yawLimitSwitchPressed = !yawLimitSwitch.get(); // DIO is active-low
    }

    @Override
    public void setFlywheelVoltage(double newVoltage) {
        flywheelMotor.setControl(voltageRequest.withOutput(newVoltage));
    }

    @Override
    public void setFlyWheelVelocity(double newVelocity, double feedforwardValue) {
        // convert meters per second to rotations per second
        flywheelMotor.setControl(
                velocityRequest
                        .withVelocity(
                                // multiply by 2 because only 1 side of ball is controlled by motor
                                Units.radiansToRotations(
                                        newVelocity / Shooter.FLYWHEEL_RADIUS_METERS * 2))
                        .withFeedForward(feedforwardValue));
    }

    @Override
    public void setPitchAngle(double newAngle) {
        double pos =
                MathUtil.inverseInterpolate(
                        Shooter.pitchMinAngleRad.get(), Shooter.pitchMaxAngleRad.get(), newAngle);
        // 1.0 - because extending actuator results in a decrease in angle
        pos = 1.0 - pos;

        pitchServo1.set(pos);
        pitchServo2.set(pos);
    }

    @Override
    public void setYawVoltage(double newVoltage) {
        yawMotor.setControl(voltageRequest.withOutput(newVoltage));
    }

    @Override
    public void setYawPosition(double newPosition) {
        setYawState(newPosition, 0, 0);
    }

    @Override
    public void setYawState(double newPosition, double newVelocity, double feedforwardValue) {
        yawMotor.setControl(
                positionRequest
                        .withPosition(newPosition)
                        .withVelocity(newVelocity)
                        .withFeedForward(feedforwardValue));
    }

    @Override
    public void updateFlywheelPID(double kP, double kD) {
        flywheelConfig.Slot0.kP = kP;
        flywheelConfig.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(
                5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));
        PhoenixUtil.tryUntilOk(
                5, () -> flywheelMotorFollower.getConfigurator().apply(flywheelConfig, 0.25));
    }

    @Override
    public void updateYawPID(double kP, double kD) {
        yawConfig.Slot0.kP = kP;
        yawConfig.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(5, () -> yawMotor.getConfigurator().apply(yawConfig, 0.25));
    }

    public void zeroYaw() {
        yawMotor.setPosition(Shooter.yawZeroingOffset.get());
    }
}
