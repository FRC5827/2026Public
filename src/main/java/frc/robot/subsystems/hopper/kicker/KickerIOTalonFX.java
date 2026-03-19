package frc.robot.subsystems.hopper.kicker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

/**
 * Kicker IO implementation for Talon FX motors (Krakens). Controls the left and right kicker
 * motors. Left motor follows right with inverted direction.
 */
public class KickerIOTalonFX implements KickerIO {

    private static final double CURRENT_LIMIT = 40.0;

    // Hardware objects
    private final TalonFX motor;

    private final TalonFXConfiguration config;

    // Status signals
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> motorTemperature;
    private final StatusSignal<AngularVelocity> motorVel;

    // Voltage control requests
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    public KickerIOTalonFX() {
        // Create motor objects
        this.motor = new TalonFX(Constants.hopperKickerMotorOpenCanbus_ID);

        // Configure both motors
        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        PhoenixUtil.tryUntilOk(5, () -> this.motor.getConfigurator().apply(config, 0.25));

        // Motor status signals
        appliedVolts = motor.getMotorVoltage();
        currentAmps = motor.getSupplyCurrent();
        motorTemperature = motor.getDeviceTemp();
        motorVel = motor.getVelocity();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, appliedVolts, currentAmps, motorTemperature, motorVel);
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        // Update kicker inputs
        var kickerStatus =
                BaseStatusSignal.refreshAll(appliedVolts, currentAmps, motorTemperature, motorVel);
        inputs.connected = kickerStatus.isOK();
        inputs.inputVoltage = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.motorTemperatureCelsius = motorTemperature.getValueAsDouble();
        inputs.angularVelRadPerSec = motorVel.getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
