package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    static final double YAW_GEAR_RATIO = 48.0; // sensor to mechanism ratio

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;

    private final Targeting targeting;

    // These values in radians
    static final LoggedTunableNumber pitchMinAngleRad =
            new LoggedTunableNumber("Turret/Pitch/Min Angle Radians", Units.degreesToRadians(20));
    static final LoggedTunableNumber pitchMaxAngleRad =
            new LoggedTunableNumber("Turret/Pitch/Max Angle Radians", Units.degreesToRadians(85));

    private static final LoggedTunableNumber yawZeroingVoltage =
            new LoggedTunableNumber("Turret/Yaw/Zeroing Voltage", 0.67);
    private static final LoggedTunableNumber yawOpenLoopVoltage =
            new LoggedTunableNumber("Turret/Yaw/Open Loop Voltage", 1.0);
    private static final LoggedTunableNumber yawTolerance =
            new LoggedTunableNumber("Turret/Yaw/Tolerance", 0.01);
    static final LoggedTunableNumber yawZeroingOffset =
            new LoggedTunableNumber("Turret/Yaw/Zeroing Offset", -0.295);
    static final LoggedTunableNumber yawMinRotations =
            new LoggedTunableNumber("Turret/Yaw/Min Rotations", -0.74);
    static final LoggedTunableNumber yawMaxRotations =
            new LoggedTunableNumber("Turret/Yaw/Max Rotations", 0.0);
    static final LoggedTunableNumber yawKP = new LoggedTunableNumber("Turret/Yaw/kP", 120.0);
    static final LoggedTunableNumber yawKD = new LoggedTunableNumber("Turret/Yaw/kD", 0.0);
    static final LoggedTunableNumber yawKS = new LoggedTunableNumber("Turret/Yaw/kS", 0.08);
    static final LoggedTunableNumber yawKV = new LoggedTunableNumber("Turret/Yaw/kV", 5.64);

    private boolean yawZeroed = false;

    private SimpleMotorFeedforward yawFeedforward =
            new SimpleMotorFeedforward(yawKS.get(), yawKV.get());

    public Turret(TurretIO io, Targeting targeting) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.targeting = targeting;
    }

    @Override
    public void periodic() {
        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);

        checkForPIDUpdates();

        if (!yawZeroed) {
            if (inputs.yawLimitSwitchPressed) {
                yawZeroed = true;
                io.setYawVoltage(0);
                io.zeroYaw();
            } else {
                io.setYawVoltage(yawZeroingVoltage.get());
            }
        }

        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret/Yaw Zeroed", yawZeroed);
        Logger.recordOutput(
                "PerformanceMonitor/Turret", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public Command rotateTurretCounterClockwise() {
        return this.run(() -> io.setYawVoltage(yawOpenLoopVoltage.get()));
    }

    public Command rotateTurretClockwise() {
        return this.run(() -> io.setYawVoltage(-yawOpenLoopVoltage.get()));
    }

    public Command raiseHood() {
        return this.runOnce(
                () -> {
                    io.setPitchAngle(pitchMinAngleRad.get());
                });
    }

    public Command lowerHood() {
        return this.runOnce(
                () -> {
                    io.setPitchAngle(pitchMaxAngleRad.get());
                });
    }

    public Command aimAtTarget() {
        return this.runEnd(
                        () -> {
                            if (yawZeroed) {
                                io.setPitchAngle(targeting.getPitchAngle());

                                if (targeting.canAimAtTarget()) {
                                    io.setYawState(
                                            targeting.getYawPosition(),
                                            targeting.getYawVelocity(),
                                            yawFeedforward.calculate(targeting.getYawVelocity()));
                                } else {
                                    io.setYawVoltage(0);
                                }
                            }
                        },
                        () -> {
                            io.setYawVoltage(0);
                        })
                .finallyDo(() -> io.setPitchAngle(pitchMaxAngleRad.get()));
    }

    public void setYawVoltage(double voltage) {
        io.setYawVoltage(voltage);
    }

    private void checkForPIDUpdates() {
        if (yawKP.hasChanged(this.hashCode()) || yawKD.hasChanged(this.hashCode())) {
            io.updateYawPID(yawKP.get(), yawKD.get());
        }

        if (yawKS.hasChanged(this.hashCode()) || yawKV.hasChanged(this.hashCode())) {
            yawFeedforward.setKs(yawKS.get());
            yawFeedforward.setKv(yawKV.get());
        }

        if (yawMinRotations.hasChanged(this.hashCode())
                || yawMaxRotations.hasChanged(this.hashCode())) {
            io.updateYawLimits(yawMinRotations.get(), yawMaxRotations.get());
        }
    }

    public boolean isYawZeroed() {
        return yawZeroed;
    }

    public boolean canAimAtPosition(double position) {
        return position >= yawMinRotations.get() && position <= yawMaxRotations.get();
    }

    public boolean isAimingAtTarget() {
        return yawZeroed
                && targeting.canAimAtTarget()
                && MathUtil.isNear(
                        targeting.getYawPosition(),
                        inputs.yawTurretPositionRotations,
                        yawTolerance.get());
    }
}
