package frc.robot.subsystems.hopper.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {

    private static final double CURRENT_LIMIT = 40.0;

    private final TalonFX motor;
    private final TalonFXConfiguration config;

    // Status signals
    public StatusSignal<Voltage> appliedVolts;
    public StatusSignal<Current> currentAmps;
    public StatusSignal<Temperature> motorTemperature;
    public StatusSignal<AngularVelocity> motorVel;

    private final VoltageOut voltageOut = new VoltageOut(0.0);

    public IndexerIOTalonFX() {
        this.motor = new TalonFX(Constants.hopperIndexerMotorOpenCanbus_ID, Constants.canivore);

        config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        PhoenixUtil.tryUntilOk(5, () -> this.motor.getConfigurator().apply(config, 0.25));

        appliedVolts = motor.getMotorVoltage();
        currentAmps = motor.getSupplyCurrent();
        motorTemperature = motor.getDeviceTemp();
        motorVel = motor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, appliedVolts, currentAmps, motorTemperature, motorVel);
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        var status =
                BaseStatusSignal.refreshAll(appliedVolts, currentAmps, motorTemperature, motorVel);
        inputs.connected = status.isOK();
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
