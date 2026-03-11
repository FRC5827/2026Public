package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
    private final DCMotorSim yawMotorSim;
    private final DCMotor yawGearbox = DCMotor.getKrakenX60(1);

    private final PIDController yawController =
            new PIDController(Turret.yawKP.get(), 0, Turret.yawKD.get());

    private double requestedPitchPosition = 0.0;
    private double requestedYawVoltage = 0.0;
    private boolean yawOpenLoop = true;
    private double yawSetpoint = 0.0;
    private boolean yawLimitSwitchPressed = true;

    public TurretIOSim() {
        yawMotorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                yawGearbox, 0.001, Turret.YAW_GEAR_RATIO),
                        yawGearbox);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        double yawAppliedVolts = requestedYawVoltage;
        if (!yawOpenLoop) {
            yawAppliedVolts =
                    yawController.calculate(yawMotorSim.getAngularPositionRotations(), yawSetpoint);
        }

        // Simulate soft limits
        double yawPositionRotations = yawMotorSim.getAngularPositionRotations();
        if (yawPositionRotations <= Turret.yawMinRotations.get() && yawAppliedVolts < 0) {
            yawAppliedVolts = 0;
        }
        if (yawPositionRotations >= Turret.yawMaxRotations.get() && yawAppliedVolts > 0) {
            yawAppliedVolts = 0;
        }

        yawMotorSim.setInputVoltage(yawAppliedVolts);
        yawMotorSim.update(0.02);

        inputs.yawConnected = true;
        inputs.pitchServoRequestedPosition = requestedPitchPosition;
        inputs.yawMotorVoltage = yawAppliedVolts;
        inputs.yawMotorCurrent = Math.abs(yawMotorSim.getCurrentDrawAmps());
        inputs.yawMotorTemperature = 0.0;
        inputs.yawTurretPositionRotations = yawMotorSim.getAngularPositionRotations();
        inputs.yawLimitSwitchPressed = yawLimitSwitchPressed;
    }

    @Override
    public void setPitchAngle(double newAngle) {
        double pos =
                MathUtil.inverseInterpolate(
                        Turret.pitchMinAngleRad.get(), Turret.pitchMaxAngleRad.get(), newAngle);
        // 1.0 - because extending actuator results in a decrease in angle (match real behavior)
        requestedPitchPosition = 1.0 - pos;
    }

    @Override
    public void setYawVoltage(double newVoltage) {
        requestedYawVoltage = newVoltage;
        yawOpenLoop = true;
    }

    @Override
    public void setYawPosition(double newPosition) {
        setYawState(newPosition, 0, 0);
    }

    @Override
    public void setYawState(double newPosition, double newVelocity, double feedforwardValue) {
        yawSetpoint =
                MathUtil.clamp(
                        newPosition, Turret.yawMinRotations.get(), Turret.yawMaxRotations.get());
        yawOpenLoop = false;
    }

    @Override
    public void updateYawPID(double kP, double kD) {
        yawController.setPID(kP, 0, kD);
    }

    @Override
    public void zeroYaw(double zeroingOffset) {
        yawMotorSim.setState(zeroingOffset, 0);
        yawLimitSwitchPressed = false;
    }
}
