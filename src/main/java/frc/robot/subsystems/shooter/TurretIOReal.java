package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class TurretIOReal implements TurretIO {
    private static final double CURRENT_LIMIT = 40.0;

    private final Servo pitchServo1, pitchServo2;

    private final TalonFXConfiguration yawConfig;
    private final TalonFX yawMotor;
    private final DigitalInput yawLimitSwitch;

    private final StatusSignal<Voltage> yawMotorVoltage;
    private final StatusSignal<Current> yawMotorCurrent;
    private final StatusSignal<Temperature> yawMotorTemperature;
    private final StatusSignal<Angle> yawMotorPosition;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    private double pitchServoPosition = 0.0;

    public TurretIOReal() {
        pitchServo1 = new Servo(Constants.turretPitchServo1PWM_ID);
        pitchServo2 = new Servo(Constants.turretPitchServo2PWM_ID);
        pitchServo1.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        pitchServo2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        yawMotor = new TalonFX(Constants.turretYawMotorCanbus_ID, Constants.canivore);
        yawLimitSwitch = new DigitalInput(Constants.turretYawLimitSwitchDIO);

        yawConfig = new TalonFXConfiguration();
        yawConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        yawConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        yawConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        yawConfig.Feedback.SensorToMechanismRatio = Turret.YAW_GEAR_RATIO;
        yawConfig.Slot0.kP = Turret.yawKP.getAsDouble();
        yawConfig.Slot0.kD = Turret.yawKD.getAsDouble();
        yawConfig.Voltage.PeakForwardVoltage = 6.0;
        yawConfig.Voltage.PeakReverseVoltage = -6.0;
        PhoenixUtil.tryUntilOk(5, () -> yawMotor.getConfigurator().apply(yawConfig, 0.25));

        yawMotorVoltage = yawMotor.getMotorVoltage();
        yawMotorCurrent = yawMotor.getSupplyCurrent();
        yawMotorTemperature = yawMotor.getDeviceTemp();
        yawMotorPosition = yawMotor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, yawMotorVoltage, yawMotorCurrent, yawMotorTemperature, yawMotorPosition);

        ParentDevice.optimizeBusUtilizationForAll(yawMotor);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        var yawStatus =
                BaseStatusSignal.refreshAll(
                        yawMotorVoltage, yawMotorCurrent, yawMotorTemperature, yawMotorPosition);

        inputs.yawConnected = yawStatus.isOK();
        inputs.yawMotorVoltage = yawMotorVoltage.getValueAsDouble();
        inputs.yawMotorCurrent = yawMotorCurrent.getValueAsDouble();
        inputs.yawMotorTemperature = yawMotorTemperature.getValueAsDouble();
        inputs.yawTurretPositionRotations = yawMotorPosition.getValueAsDouble();
        inputs.yawLimitSwitchPressed = !yawLimitSwitch.get();
        inputs.pitchServoRequestedPosition = pitchServoPosition;
    }

    @Override
    public void setPitchAngle(double newAngle) {
        double pos =
                MathUtil.inverseInterpolate(
                        Turret.pitchMinAngleRad.get(), Turret.pitchMaxAngleRad.get(), newAngle);
        // 1.0 - because extending actuator results in a decrease in angle
        pos = 1.0 - pos;

        pitchServo1.set(pos);
        pitchServo2.set(pos);
        pitchServoPosition = pos;
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
    public void updateYawPID(double kP, double kD) {
        yawConfig.Slot0.kP = kP;
        yawConfig.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(5, () -> yawMotor.getConfigurator().apply(yawConfig, 0.25));
    }

    @Override
    public void updateYawLimits(double minRotations, double maxRotations) {
        yawConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minRotations;
        yawConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRotations;
        PhoenixUtil.tryUntilOk(5, () -> yawMotor.getConfigurator().apply(yawConfig, 0.25));
    }

    @Override
    public void zeroYaw() {
        yawConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        yawConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Turret.yawMinRotations.get();
        yawConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        yawConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Turret.yawMaxRotations.get();
        PhoenixUtil.tryUntilOk(5, () -> yawMotor.getConfigurator().apply(yawConfig, 0.25));
        yawMotor.setPosition(Turret.yawZeroingOffset.get());
    }
}
