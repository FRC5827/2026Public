package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    static final double FLYWHEEL_GEAR_RATIO = 1.0; // sensor to mechanism ratio
    static final double YAW_GEAR_RATIO = 48.0; // sensor to mechanism ratio
    static final double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    // for command requirements, since shooter has multiple motors and we want to be able to
    // run them independently
    private final Subsystem shooterSubsystem = new Subsystem() {};

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;
    private final Supplier<Pose2d> robotPoseSupplier;

    // translation of shooter from robot center (m) (please update please please)
    private final Translation3d shooterTranslationOnRobot =
            new Translation3d(
                    Units.inchesToMeters(6.5),
                    Units.inchesToMeters(-4.5),
                    Units.inchesToMeters(16));

    private static final LoggedTunableNumber hubEdgeDY =
            new LoggedTunableNumber("Shooter/Hub Edge dy", 0.3);
    private static final LoggedTunableNumber hubEdgeDX =
            new LoggedTunableNumber("Shooter/Hub Edge dx", 0.84);

    private static final LoggedTunableNumber flywheelOpenLoopVoltage =
            new LoggedTunableNumber("Shooter/Flywheel/Open Loop Voltage", 0.5);
    private static final LoggedTunableNumber flywheelTolerance =
            new LoggedTunableNumber("Shooter/Flywheel/ToleranceMPS", 0.1);
    static final LoggedTunableNumber flywheelKP =
            new LoggedTunableNumber("Shooter/Flywheel/kP", 0.02);
    static final LoggedTunableNumber flywheelKD =
            new LoggedTunableNumber("Shooter/Flywheel/kD", 0.0);
    static final LoggedTunableNumber flywheelKS =
            new LoggedTunableNumber("Shooter/Flywheel/kS", 0.0);
    static final LoggedTunableNumber flywheelKV =
            new LoggedTunableNumber("Shooter/Flywheel/kV", 0.76);
    // multiplier to account for lack of acceleration under hood
    static final LoggedTunableNumber flywheelMultiplier =
            new LoggedTunableNumber("Shooter/Flywheel/Multiplier", 1.2);

    // These values in radians
    static final LoggedTunableNumber pitchMinAngleRad =
            new LoggedTunableNumber("Shooter/Pitch/Min Angle Radians", Units.degreesToRadians(20));
    static final LoggedTunableNumber pitchMaxAngleRad =
            new LoggedTunableNumber("Shooter/Pitch/Max Angle Radians", Units.degreesToRadians(85));

    private static final LoggedTunableNumber yawZeroingVoltage =
            new LoggedTunableNumber("Shooter/Yaw/Zeroing Voltage", 0.67);
    private static final LoggedTunableNumber yawOpenLoopVoltage =
            new LoggedTunableNumber("Shooter/Yaw/Open Loop Voltage", 1.0);
    private static final LoggedTunableNumber yawTolerance =
            new LoggedTunableNumber("Shooter/Yaw/Tolerance", 0.01);
    static final LoggedTunableNumber yawZeroingOffset =
            new LoggedTunableNumber("Shooter/Yaw/Zeroing Offset", -0.285);
    static final LoggedTunableNumber yawMinRotations =
            new LoggedTunableNumber("Shooter/Yaw/Min Rotations", -0.4);
    static final LoggedTunableNumber yawMaxRotations =
            new LoggedTunableNumber("Shooter/Yaw/Max Rotations", 0.13);
    static final LoggedTunableNumber yawKP = new LoggedTunableNumber("Shooter/Yaw/kP", 120.0);
    static final LoggedTunableNumber yawKD = new LoggedTunableNumber("Shooter/Yaw/kD", 0.0);
    static final LoggedTunableNumber yawKS = new LoggedTunableNumber("Shooter/Yaw/kS", 0.0);
    static final LoggedTunableNumber yawKV = new LoggedTunableNumber("Shooter/Yaw/kV", 0.76);

    private Translation3d targetTranslation = null;
    // clearance is defined as an additional horizontal and vertical distance from the target
    // that the ball must clear before reaching the target
    private Translation2d targetClearance = null;
    // coefficients for quadratic equation to model pitch and velocity
    private double[] pitchAndVelocityCoefficients = null;
    private double flywheelTargetVelocity = 0.0;
    private double pitchTargetAngle = pitchMaxAngleRad.get();
    private double yawTargetPosition = 0.0;
    private double yawTargetVelocity = 0.0;
    private boolean yawZeroed = false;
    private boolean canAimAtTarget = false;
    private Pose2d lastPose;
    private double lastPoseTimestamp;

    private SimpleMotorFeedforward flywheelFeedforward =
            new SimpleMotorFeedforward(flywheelKS.get(), flywheelKV.get());
    private SimpleMotorFeedforward yawFeedforward =
            new SimpleMotorFeedforward(yawKS.get(), yawKV.get());

    public Shooter(ShooterIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.inputs = new ShooterIOInputsAutoLogged();
        this.robotPoseSupplier = robotPoseSupplier;
        this.lastPose = robotPoseSupplier.get();
        this.lastPoseTimestamp = Timer.getFPGATimestamp();
    }

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
        } else if (targetTranslation != null && targetClearance != null) {
            calculateVelocitiesForTarget(targetTranslation);
            io.setYawState(
                    yawTargetPosition,
                    yawTargetVelocity,
                    yawFeedforward.calculate(yawTargetVelocity));
        } else {
            // leave yaw in previous position TODO might change later
            io.setYawVoltage(0.0);
        }
        io.setPitchAngle(pitchTargetAngle);

        // update lastPose and lastPoseTimestamp
        lastPose = robotPoseSupplier.get();
        lastPoseTimestamp = Timer.getFPGATimestamp();

        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/Yaw Zeroed", yawZeroed);
        Logger.recordOutput("Shooter/Flywheel Target Velocity", flywheelTargetVelocity);
        Logger.recordOutput(
                "Shooter/Flywheel Target Without Multiplier",
                flywheelTargetVelocity / flywheelMultiplier.get());
        Logger.recordOutput("Shooter/Pitch Target Angle", pitchTargetAngle);
        Logger.recordOutput("Shooter/Yaw Target Position", yawTargetPosition);
        Logger.recordOutput("Shooter/Yaw Target Velocity", yawTargetVelocity);
        Logger.recordOutput("Shooter/Target Translation", targetTranslation);
        Logger.recordOutput("Shooter/Can Aim At Target", canAimAtTarget);
        Logger.recordOutput("Shooter/Is Aimed At Hub", isAimedAtTarget());
        Logger.recordOutput(
                "Shooter/Pitch and Velocity Coefficients", pitchAndVelocityCoefficients);
        Logger.recordOutput(
                "PerformanceMonitor/Shooter", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public Command shoot() {
        return Commands.startEnd(
                () -> {
                    io.setFlywheelVoltage(flywheelOpenLoopVoltage.get());
                },
                () -> {
                    io.setFlywheelVoltage(0);
                },
                shooterSubsystem);
    }

    public Command shootAtTarget() {
        return Commands.runEnd(
                () -> {
                    io.setFlyWheelVelocity(
                            flywheelTargetVelocity,
                            flywheelFeedforward.calculate(flywheelTargetVelocity));
                },
                () -> {
                    io.setFlywheelVoltage(0);
                },
                shooterSubsystem);
    }

    public Command raiseShooterHood() {
        return Commands.runOnce(
                () -> {
                    pitchTargetAngle = pitchMinAngleRad.get();
                },
                this);
    }

    public Command lowerShooterHood() {
        return Commands.runOnce(
                () -> {
                    pitchTargetAngle = pitchMaxAngleRad.get();
                },
                this);
    }

    public Command rotateTurretCounterClockwise() {
        return Commands.run(() -> io.setYawVoltage(yawOpenLoopVoltage.get()), this);
    }

    public Command rotateTurretClockwise() {
        return Commands.run(() -> io.setYawVoltage(-yawOpenLoopVoltage.get()), this);
    }

    public Command aimAtHub() {
        return Commands.runOnce(
                        () -> {
                            targetClearance = new Translation2d(hubEdgeDX.get(), hubEdgeDY.get());
                        },
                        this)
                .andThen(
                        Commands.runEnd(
                                () -> {
                                    if (robotPoseSupplier.get().getX()
                                                    > FieldConstants.LinesVertical.neutralZoneNear
                                            && robotPoseSupplier.get().getX()
                                                    < FieldConstants.LinesVertical.neutralZoneFar) {
                                        targetTranslation =
                                                null; // don't aim if we're in the neutral zone
                                    } else {
                                        Translation3d hubTranslation =
                                                AllianceFlipUtil.shouldFlip()
                                                        ? FieldConstants.Hub.oppInnerCenterPoint
                                                        : FieldConstants.Hub.innerCenterPoint;
                                        targetTranslation = hubTranslation;
                                    }
                                },
                                () -> {
                                    targetTranslation = null;
                                    targetClearance = null;
                                },
                                this))
                .andThen(this::lowerShooterHood);
    }

    private void checkForPIDUpdates() {
        if (flywheelKP.hasChanged(this.hashCode()) || flywheelKD.hasChanged(this.hashCode())) {
            io.updateFlywheelPID(flywheelKP.get(), flywheelKD.get());
        }

        if (flywheelKS.hasChanged(this.hashCode()) || flywheelKV.hasChanged(this.hashCode())) {
            flywheelFeedforward.setKs(flywheelKS.get());
            flywheelFeedforward.setKv(flywheelKV.get());
        }

        if (yawKP.hasChanged(this.hashCode()) || yawKD.hasChanged(this.hashCode())) {
            io.updateYawPID(yawKP.get(), yawKD.get());
        }

        if (yawKS.hasChanged(this.hashCode()) || yawKV.hasChanged(this.hashCode())) {
            yawFeedforward.setKs(yawKS.get());
            yawFeedforward.setKv(yawKV.get());
        }
    }

    private double calculateYawPosition(
            Pose2d robotPose, Translation2d shooterToTargetTranslation) {
        // Calculate angle from shooter to target
        Rotation2d targetAngle = shooterToTargetTranslation.getAngle();

        // Convert to robot-relative angle (how far the turret needs be rotated from robot forward)
        Rotation2d calculatedYawPosition = targetAngle.minus(robotPose.getRotation());

        return calculatedYawPosition.getRotations();
    }

    private double[] solveQuadraticSystem(
            double x1, double y1, double x2, double y2, double x3, double y3) {
        try {
            double[] vec =
                    new Matrix<>(
                                    Nat.N3(),
                                    Nat.N3(),
                                    new double[] {
                                        x1 * x1, x1, 1,
                                        x2 * x2, x2, 1,
                                        x3 * x3, x3, 1
                                    })
                            .solve(new Matrix<>(Nat.N3(), Nat.N1(), new double[] {y1, y2, y3}))
                            .getData();

            // some sanity checks, d^2y/dx^2 should be negative, dy/dx should be positive at 0
            if (vec[0] >= 0 || vec[1] <= 0) {
                return null;
            }
            return vec;
        } catch (Exception e) {
            return null;
        }
    }

    private double[] solveForPitchAndVelocity(double a, double b, double c) {
        if (c != 0) {
            // this should never happen since we solve the system with c = 0, but just in case
            return new double[] {0, 0};
        }

        // curvePeakX = -b / 2a
        // curvePeakY = a * (curvePeakX)^2 + b * curvePeakX
        //            = b^2 / 4a - b^2 / 2a
        //            = -b^2 / 4a
        double curvePeakY = -(b * b) / (4 * a);

        // b for y(t) equation is verticalVelocity = v
        // peakX for y(t) = v / g
        // peakY for y(t) = -(g/2) * (v/g)^2 + v * (v/g)
        //                = -v^2 / 2g + v^2 / g
        //                = v^2 / 2g
        // since the peakYs are equal
        // peakY = v^2 / 2g
        // v = sqrt(2 * peakY * g)
        double verticalVelocity =
                Math.sqrt(2 * curvePeakY * FieldConstants.PhysicalConstants.GRAVITY);

        // can be found with dy/dx = b
        // dx = dy / b
        double horizontalVelocity = verticalVelocity / b;

        return new double[] {
            Math.atan2(verticalVelocity, horizontalVelocity),
            Math.hypot(horizontalVelocity, verticalVelocity)
        };
    }

    private double solveForVelocityWithAngle(
            double angle, double horizontalDistance, double verticalDistance) {
        // y = ax^2 + bx
        // b = tan(angle)
        // plug in x = horizontalDistance and y = verticalDistance to get a
        // a = (y - b*x) / x^2
        double b = Math.tan(angle);
        double a =
                (verticalDistance - b * horizontalDistance)
                        / (horizontalDistance * horizontalDistance);

        return solveForPitchAndVelocity(a, b, 0)[1];
    }

    private void calculateVelocitiesForTarget(Translation3d targetTranslation) {
        Pose2d robotPose = robotPoseSupplier.get();

        Translation3d shooterTranslation =
                new Translation3d(robotPose.getTranslation()).plus(shooterTranslationOnRobot);
        // Calculate translation from shooter to hub
        Translation3d shooterToHubTranslation = targetTranslation.minus(shooterTranslation);

        Translation2d shooterToHubHorizontal = shooterToHubTranslation.toTranslation2d();
        double shooterToHubVertical = shooterToHubTranslation.getZ();

        // TODO: maybe estimate time to target and combine with last pose to adjust target
        // if shooting while moving is too inaccurate

        double dist = shooterToHubHorizontal.getNorm();
        // these points are in coordinate system where x is horizontal distance and y is vertical
        // distance
        pitchAndVelocityCoefficients =
                solveQuadraticSystem(
                        0, // shooter x
                        0, // shooter y
                        dist - targetClearance.getX(), // lip of hub x
                        shooterToHubVertical + targetClearance.getY(), // lip of hub y
                        dist, // center of hub x
                        shooterToHubVertical); // center of hub y

        pitchTargetAngle = 0;
        if (pitchAndVelocityCoefficients != null) {
            double[] values =
                    solveForPitchAndVelocity(
                            pitchAndVelocityCoefficients[0],
                            pitchAndVelocityCoefficients[1],
                            pitchAndVelocityCoefficients[2]);
            pitchTargetAngle = values[0];

            if (pitchTargetAngle < pitchMinAngleRad.get()) {
                pitchTargetAngle = pitchMinAngleRad.get();
                flywheelTargetVelocity =
                        solveForVelocityWithAngle(pitchTargetAngle, dist, shooterToHubVertical);
            } else if (pitchTargetAngle <= pitchMaxAngleRad.get()) {
                flywheelTargetVelocity = values[1];
            } else {
                // cannot hit target
                pitchTargetAngle = pitchMaxAngleRad.get();
                flywheelTargetVelocity = 0;
            }

            // in case calculated velocity is negative, don't shoot
            flywheelTargetVelocity = Math.max(flywheelTargetVelocity, 0);
        } else {
            flywheelTargetVelocity = 0;
        }

        flywheelTargetVelocity *= flywheelMultiplier.get();

        double newYawTargetPosition = calculateYawPosition(robotPose, shooterToHubHorizontal);

        canAimAtTarget =
                newYawTargetPosition > yawMinRotations.get()
                        && newYawTargetPosition < yawMaxRotations.get()
                        && pitchTargetAngle != 0;

        // avoid moving turret if we can't aim at target to prevent unnecessary movement
        if (newYawTargetPosition >= yawMinRotations.get()
                && newYawTargetPosition <= yawMaxRotations.get()) {
            yawTargetPosition = newYawTargetPosition;

            Twist2d deltaPose = lastPose.log(robotPose);
            yawTargetVelocity =
                    Units.radiansToRotations(
                            -deltaPose.dtheta / (Timer.getFPGATimestamp() - lastPoseTimestamp));
        } else {
            yawTargetVelocity = 0;
        }
    }

    public boolean shooterRunningAtVelocity() {
        return flywheelTargetVelocity > 0
                && MathUtil.isNear(
                        flywheelTargetVelocity,
                        inputs.flywheelMotorVelocityMPS,
                        flywheelTolerance.get());
    }

    public boolean isAimedAtTarget() {
        return canAimAtTarget
                && MathUtil.isNear(
                        yawTargetPosition,
                        inputs.yawTurretPositionRotations,
                        yawTolerance.get()); // no feedback from pitch;
    }
}
