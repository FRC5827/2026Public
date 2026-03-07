package frc.robot.subsystems.intake.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {

    private static final double CURRENT_LIMIT = 40.0; // subject to change

    private final TalonFX flywheelMotor;
    private final TalonFXConfiguration config;
    public StatusSignal<Voltage> flywheelAppliedVolts;
    public StatusSignal<Current> flywheelCurrentAmps;
    public StatusSignal<Temperature> flywheelMotorTemperature;
    public StatusSignal<AngularAcceleration> flywheelMotorAccel;
    public StatusSignal<AngularVelocity> flywheelMotorVel;

    private final VoltageOut flywheelVoltageOut = new VoltageOut(0.0);

    public FlywheelIOTalonFX() {
        this.flywheelMotor =
                new TalonFX(Constants.intakeFlywheelMotorCanbus_ID, Constants.canivore);
        config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        PhoenixUtil.tryUntilOk(5, () -> this.flywheelMotor.getConfigurator().apply(config, 0.25));

        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelCurrentAmps = flywheelMotor.getSupplyCurrent();
        flywheelMotorTemperature = flywheelMotor.getDeviceTemp();
        flywheelMotorAccel = flywheelMotor.getAcceleration();
        flywheelMotorVel = flywheelMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                flywheelAppliedVolts,
                flywheelCurrentAmps,
                flywheelMotorTemperature,
                flywheelMotorAccel,
                flywheelMotorVel);
        ParentDevice.optimizeBusUtilizationForAll(flywheelMotor);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        var flywheelStatus =
                BaseStatusSignal.refreshAll(
                        flywheelAppliedVolts,
                        flywheelCurrentAmps,
                        flywheelMotorTemperature,
                        flywheelMotorAccel,
                        flywheelMotorVel);

        inputs.connected = flywheelStatus.isOK();
        inputs.inputVoltage = flywheelAppliedVolts.getValueAsDouble();
        inputs.currentAmps = flywheelCurrentAmps.getValueAsDouble();
        inputs.motorTemperatureCelsius = flywheelMotorTemperature.getValueAsDouble();
        inputs.angularAccelRadPerSecSq = flywheelMotorAccel.getValueAsDouble();
        inputs.angularVelRadPerSec = flywheelMotorVel.getValueAsDouble();
    }

    @Override
    public void setFlywheelVoltage(double voltage) {
        flywheelMotor.setControl(flywheelVoltageOut.withOutput(voltage));
    }

    @Override
    public double getFlywheelVelocity(FlywheelIOInputs inputs) {
        return inputs.angularVelRadPerSec;
    }

    @Override
    public void setFlywheelBrakeMode(boolean brake) {
        flywheelMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
