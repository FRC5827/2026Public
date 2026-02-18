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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;

import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {
    private static final double CURRENT_LIMIT = 40.0; // amps

    private final TalonFX firingMotor, firingMotorFollower;
    private final TalonFX yawMotor;
    private final Servo actuatorLeft, actuatorRight;
    private final DigitalInput yawLimitSwitch;
    private final TalonFXConfiguration config;

    private boolean yawZeroed = false;

    private StatusSignal<AngularVelocity> firingAngularVelocity;

    private StatusSignal<Voltage> firingMotorVoltage;
    private StatusSignal<Voltage> firingMotorFollowerVoltage;

    private StatusSignal<Current> firingMotorCurrentAmps;
    private StatusSignal<Current> firingMotorFollowerCurrentAmps;

    private StatusSignal<Temperature> firingMotorTemperature;
    private StatusSignal<Temperature> firingMotorFollowerTemperature;

    private StatusSignal<Voltage> yawMotorVoltage;
    private StatusSignal<Current> yawMotorCurrentAmps;
    private StatusSignal<Temperature> yawMotorTemperature;
    private StatusSignal<Angle> yawMotorPosition;

    private final VoltageOut firingVoltageOut = new VoltageOut(0.0);
    private final VoltageOut yawVoltageOut = new VoltageOut(0.0);
    private final PositionVoltage yawPositionRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    private double actuatorPosition = 0.0;

    public ShooterIOReal() {
        firingMotor =
                new TalonFX(Constants.shooterFiringMotorOpenCanbus_ID, TunerConstants.kCANBus);
        firingMotorFollower =
                new TalonFX(
                        Constants.shooterFiringMotorFollowerOpenCanbus_ID, TunerConstants.kCANBus);
        yawMotor = new TalonFX(Constants.shooterYawMotorCanbus_ID, TunerConstants.kCANBus);

        actuatorLeft = new Servo(0);
        actuatorRight = new Servo(1);

        yawLimitSwitch = new DigitalInput(Constants.yawLimitSwitchDIO);

        firingMotorFollower.setControl(
                new Follower(firingMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        config.Slot0.kP = Shooter.firingMotorP.getAsDouble();
        config.Slot0.kD = Shooter.firingMotorD.getAsDouble();

        PhoenixUtil.tryUntilOk(5, () -> firingMotor.getConfigurator().apply(config, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> firingMotorFollower.getConfigurator().apply(config, 0.25));

        TalonFXConfiguration yawConfig = new TalonFXConfiguration();
        yawConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        yawConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        yawConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        yawConfig.Slot0.kP = Shooter.yawMotorP.getAsDouble();
        yawConfig.Slot0.kD = Shooter.yawMotorD.getAsDouble();
        yawConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        yawConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Shooter.yawMinRotations.get();
        yawConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        yawConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Shooter.yawMaxRotations.get();
        PhoenixUtil.tryUntilOk(5, () -> yawMotor.getConfigurator().apply(yawConfig, 0.25));

        this.firingAngularVelocity = firingMotor.getVelocity();

        this.firingMotorVoltage = firingMotor.getMotorVoltage();
        this.firingMotorFollowerVoltage = firingMotorFollower.getMotorVoltage();

        this.firingMotorCurrentAmps = firingMotor.getSupplyCurrent();
        this.firingMotorFollowerCurrentAmps = firingMotorFollower.getSupplyCurrent();

        this.firingMotorTemperature = firingMotor.getDeviceTemp();
        this.firingMotorFollowerTemperature = firingMotorFollower.getDeviceTemp();

        this.yawMotorVoltage = yawMotor.getMotorVoltage();
        this.yawMotorCurrentAmps = yawMotor.getSupplyCurrent();
        this.yawMotorTemperature = yawMotor.getDeviceTemp();
        this.yawMotorPosition = yawMotor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                firingAngularVelocity,
                firingMotorVoltage,
                firingMotorFollowerVoltage,
                firingMotorCurrentAmps,
                firingMotorFollowerCurrentAmps,
                firingMotorTemperature,
                firingMotorFollowerTemperature,
                yawMotorVoltage,
                yawMotorCurrentAmps,
                yawMotorTemperature,
                yawMotorPosition);

        ParentDevice.optimizeBusUtilizationForAll(firingMotor);
        ParentDevice.optimizeBusUtilizationForAll(firingMotorFollower);
        ParentDevice.optimizeBusUtilizationForAll(yawMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var shooterStatus =
                BaseStatusSignal.refreshAll(
                        firingAngularVelocity,
                        firingMotorVoltage,
                        firingMotorFollowerVoltage,
                        firingMotorCurrentAmps,
                        firingMotorFollowerCurrentAmps,
                        firingMotorTemperature,
                        firingMotorFollowerTemperature,
                        yawMotorVoltage,
                        yawMotorCurrentAmps,
                        yawMotorTemperature,
                        yawMotorPosition);
        inputs.shooterConnected = shooterStatus.isOK();

        // Firing motor telemetry
        inputs.firingAngularVelocityRPS = firingAngularVelocity.getValue().in(RadiansPerSecond);
        inputs.firingMotorAppliedVolts = firingMotorVoltage.getValueAsDouble();
        inputs.firingMotorFollowerAppliedVolts = firingMotorFollowerVoltage.getValueAsDouble();
        inputs.firingMotorCurrentAmps = firingMotorCurrentAmps.getValueAsDouble();
        inputs.firingMotorFollowerCurrentAmps = firingMotorFollowerCurrentAmps.getValueAsDouble();
        inputs.firingMotorTemperature = firingMotorTemperature.getValueAsDouble();
        inputs.firingMotorFollowerTemperature = firingMotorFollowerTemperature.getValueAsDouble();

        // Yaw motor telemetry
        inputs.yawControlMotorAppliedVolts = yawMotorVoltage.getValueAsDouble();
        inputs.yawControlMotorCurrentAmps = yawMotorCurrentAmps.getValueAsDouble();
        inputs.yawControlMotorTemperature = yawMotorTemperature.getValueAsDouble();
        inputs.shooterPositionYaw = yawMotorPosition.getValueAsDouble();

        // Actuator telemetry
        inputs.actuatorPosition = actuatorPosition;

        // Limit switch zeroing
        boolean limitPressed = !yawLimitSwitch.get(); // DIO is active-low
        inputs.yawLimitSwitchPressed = limitPressed;
        if (limitPressed && !yawZeroed) {
            yawMotor.setPosition(0);
            yawZeroed = true;
        }
        inputs.yawEncoderZeroed = yawZeroed;
    }

    @Override
    public void setTurretPitchPosition(double newPosition) {
        setActuatorPosition(newPosition);
    }

    @Override
    public void setTurretYawPosition(double newPosition) {}

    @Override
    public void setTurretFiringVoltage(double newVoltage) {
        firingVoltageOut.withOutput(MathUtil.clamp(newVoltage, -1, 1));
        firingMotor.setControl(firingVoltageOut);
    }

    @Override
    public void setTurretTargetFiringVelocity(double newVelocity) {
        velocityRequest.withVelocity(MathUtil.clamp(newVelocity, -0.2, 0.2));
        firingMotor.setControl(velocityRequest);
    }

    @Override
    public void setTurretPitchMotorVoltage(double newVoltage) {}

    @Override
    public void setTurretYawMotorVoltage(double newVoltage) {
        if (yawZeroed) {
            double motorRotations = yawMotorPosition.getValueAsDouble();
            double minRot = Shooter.yawMinRotations.get();
            double maxRot = Shooter.yawMaxRotations.get();

            Logger.recordOutput("Shooter/yawLimitDebug/motorRotations", motorRotations);
            Logger.recordOutput("Shooter/yawLimitDebug/requestedVoltage", newVoltage);

            // At forward limit: hold position instead of allowing further rotation
            if (motorRotations >= maxRot && newVoltage > 0) {
                Logger.recordOutput("Shooter/yawLimitDebug/holding", "forward");
                yawMotor.setControl(yawPositionRequest.withPosition(maxRot));
                return;
            }
            // At reverse limit: hold position instead of allowing further rotation
            if (motorRotations <= minRot && newVoltage < 0) {
                Logger.recordOutput("Shooter/yawLimitDebug/holding", "reverse");
                yawMotor.setControl(yawPositionRequest.withPosition(minRot));
                return;
            }

            Logger.recordOutput("Shooter/yawLimitDebug/holding", "none");
        }
        yawVoltageOut.withOutput(MathUtil.clamp(newVoltage, -2, 2));
        yawMotor.setControl(yawVoltageOut);
    }

    @Override
    public void setActuatorPosition(double position) {
        actuatorPosition = position;
        actuatorLeft.set(position);
        actuatorRight.set(position);
    }

    @Override
    public void updatePIDFiringMotors(double kP, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kD = kD;
        PhoenixUtil.tryUntilOk(5, () -> firingMotor.getConfigurator().apply(config, 0.25));
    }

    @Override
    public void updatePIDPitchMotor(double kP, double kD) {}

    @Override
    public void updatePIDYawMotor(double kP, double kD) {}
}
