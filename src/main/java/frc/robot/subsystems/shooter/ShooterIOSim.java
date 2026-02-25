package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {

    private final DCMotorSim flywheelMotorSim;
    private final DCMotorSim flywheelMotorFollowerSim;
    private final DCMotorSim yawMotorSim;

    private final DCMotor flywheelGearbox = DCMotor.getKrakenX60(1);
    private final DCMotor yawGearbox = DCMotor.getKrakenX60(1);

    private static final double MAX_VOLTAGE = 12.0;

    private final PIDController yawController =
            new PIDController(Shooter.yawKP.get(), 0, Shooter.yawKD.get());
    private final PIDController flywheelController =
            new PIDController(Shooter.flywheelKP.get(), 0, Shooter.flywheelKD.get());

    // Open-loop voltage requests
    private double requestedFlywheelVoltage = 0.0;
    private double requestedPitchPosition = 0.0;
    private double requestedYawVoltage = 0.0;

    private boolean flywheelOpenLoop = true;
    private boolean yawOpenLoop = true;

    public ShooterIOSim() {
        flywheelMotorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                flywheelGearbox, 0.001, Shooter.FLYWHEEL_GEAR_RATIO),
                        flywheelGearbox);

        flywheelMotorFollowerSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                flywheelGearbox, 0.001, Shooter.FLYWHEEL_GEAR_RATIO),
                        flywheelGearbox);

        yawMotorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                yawGearbox, 0.001, Shooter.YAW_GEAR_RATIO),
                        yawGearbox);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelMotorSim.update(0.02);
        flywheelMotorFollowerSim.update(0.02);
        yawMotorSim.update(0.02);

        // Flywheel control
        double flywheelAppliedVolts;
        if (flywheelOpenLoop) {
            flywheelAppliedVolts = requestedFlywheelVoltage;
        } else {
            // PID operates in rotations/sec to match TalonFX VelocityVoltage convention
            double currentVelocityRotPerSec =
                    Units.radiansToRotations(flywheelMotorSim.getAngularVelocityRadPerSec());
            flywheelAppliedVolts =
                    MathUtil.clamp(
                            flywheelController.calculate(currentVelocityRotPerSec),
                            -MAX_VOLTAGE,
                            MAX_VOLTAGE);
        }
        flywheelMotorSim.setInputVoltage(flywheelAppliedVolts);
        flywheelMotorFollowerSim.setInputVoltage(-flywheelAppliedVolts);

        inputs.flywheelConnected = true;
        inputs.flywheelMotorVoltage = flywheelAppliedVolts;
        inputs.flywheelMotorVoltageFollower = -flywheelAppliedVolts;
        inputs.flywheelMotorCurrent = flywheelMotorSim.getCurrentDrawAmps();
        inputs.flywheelMotorCurrentFollower = flywheelMotorFollowerSim.getCurrentDrawAmps();
        inputs.flywheelMotorTemp = 0.0;
        inputs.flywheelMotorTempFollower = 0.0;
        // mechanism angular velocity (rad/s) * wheel radius = surface speed (m/s)
        inputs.flywheelMotorVelocityMPS =
                flywheelMotorSim.getAngularVelocityRadPerSec() * Shooter.FLYWHEEL_RADIUS_METERS;

        inputs.pitchServoRequestedPosition = requestedPitchPosition;

        // Yaw control
        double yawAppliedVolts;
        if (yawOpenLoop) {
            yawAppliedVolts = requestedYawVoltage;
        } else {
            // PID operates in rotations to match TalonFX PositionVoltage convention
            yawAppliedVolts =
                    MathUtil.clamp(
                            yawController.calculate(yawMotorSim.getAngularPositionRotations()),
                            -MAX_VOLTAGE,
                            MAX_VOLTAGE);
        }
        // Enforce soft limits: don't allow voltage that would push past the limit
        double yawPositionRotations = yawMotorSim.getAngularPositionRotations();
        if (yawPositionRotations <= Shooter.yawMinRotations.get() && yawAppliedVolts < 0) {
            yawAppliedVolts = 0;
        }
        if (yawPositionRotations >= Shooter.yawMaxRotations.get() && yawAppliedVolts > 0) {
            yawAppliedVolts = 0;
        }
        yawMotorSim.setInputVoltage(yawAppliedVolts);

        inputs.yawConnected = true;
        inputs.yawMotorVoltage = yawAppliedVolts;
        inputs.yawMotorCurrent = yawMotorSim.getCurrentDrawAmps();
        inputs.yawMotorTemperature = 0.0;
        inputs.yawTurretPositionRotations = yawPositionRotations;
        inputs.yawLimitSwitchPressed = false;
    }

    @Override
    public void setFlywheelVoltage(double newVoltage) {
        flywheelOpenLoop = true;
        requestedFlywheelVoltage = MathUtil.clamp(newVoltage, -MAX_VOLTAGE, MAX_VOLTAGE);
    }

    @Override
    public void setFlyWheelVelocity(double newVelocity, double feedforwardValue) {
        flywheelOpenLoop = false;
        // newVelocity is in m/s, convert to rot/s to match TalonFX VelocityVoltage convention
        flywheelController.setSetpoint(
                Units.radiansToRotations(newVelocity / Shooter.FLYWHEEL_RADIUS_METERS));
    }

    @Override
    public void setYawVoltage(double newVoltage) {
        yawOpenLoop = true;
        requestedYawVoltage = MathUtil.clamp(newVoltage, -MAX_VOLTAGE, MAX_VOLTAGE);
    }

    @Override
    public void setYawPosition(double newPosition) {
        setYawState(newPosition, 0, 0);
    }

    @Override
    public void setYawState(double newPosition, double newVelocity, double feedforwardValue) {
        yawOpenLoop = false;
        // Clamp to the same limits used by the real robot's hardware soft limits
        double clampedPosition =
                MathUtil.clamp(
                        newPosition, Shooter.yawMinRotations.get(), Shooter.yawMaxRotations.get());
        yawController.setSetpoint(clampedPosition);
    }

    @Override
    public void setPitchAngle(double newAngle) {
        double pos =
                MathUtil.inverseInterpolate(
                        Shooter.pitchMinAngleRad.get(), Shooter.pitchMaxAngleRad.get(), newAngle);
        // 1.0 - because extending actuator results in a decrease in angle
        pos = 1.0 - pos;

        requestedPitchPosition = pos;
    }

    @Override
    public void updateFlywheelPID(double kP, double kD) {
        flywheelController.setPID(kP, 0, kD);
    }

    @Override
    public void updateYawPID(double kP, double kD) {
        yawController.setPID(kP, 0, kD);
    }

    @Override
    public void updateYawLimits(double minRotations, double maxRotations) {
        // Soft limits are enforced dynamically in updateInputs() using Shooter tunables,
        // so no state needs to be stored here.
    }
}
