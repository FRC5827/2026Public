package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs;

    private final Translation3d shooterTranslationOnRobot =
            new Translation3d(Units.inchesToMeters(13.0), 0, 0.8);
    // translation of shooter from robot center (m) (please update please please)
    private static final double flywheelRadiusMeters = 0.05; // ~2 inch radius, measure on robot
    private static final LoggedTunableNumber targetFlywheelRadsPerSecond =
            new LoggedTunableNumber("Shooter/targetFlywheelRadiansPerSecond", 0.2);
    private static final LoggedTunableNumber pitchMinRad =
            new LoggedTunableNumber("Shooter/pitchMinRad", 0.0);
    private static final LoggedTunableNumber pitchMaxRad =
            new LoggedTunableNumber("Shooter/pitchMaxRad", Math.PI / 4.0);

    private boolean aimingAtHub =
            false; // whether we are currently trying to aim at the hub (used for aiming command)

    private final Supplier<Pose2d> robotPose;
    private static final LoggedTunableNumber maxShootingSpeed = // RPM
            new LoggedTunableNumber("Shooter/maxShootingSpeed", 120);
    private static final LoggedTunableNumber shooterTrenchPitchPosition =
            new LoggedTunableNumber("Shooter/trenchPitchPosition", 0);

    static final LoggedTunableNumber firingMotorP =
            new LoggedTunableNumber("Shooter/firingMotorPID/p", 0.1);
    static final LoggedTunableNumber firingMotorD =
            new LoggedTunableNumber("Shooter/firingMotorPID/d", 0);
    static final LoggedTunableNumber yawMotorP =
            new LoggedTunableNumber("Shooter/yawMotorPID/p", 1.0);
    static final LoggedTunableNumber yawMotorD =
            new LoggedTunableNumber("Shooter/yawMotorPID/d", 0);

    private static final LoggedTunableNumber maxYawVoltage =
            new LoggedTunableNumber("Shooter/maxYawVoltage", 6.0);
    static final LoggedTunableNumber yawMinRotations =
            new LoggedTunableNumber("Shooter/yawMinRotations", -10.0);
    static final LoggedTunableNumber yawMaxRotations =
            new LoggedTunableNumber("Shooter/yawMaxRotations", 18.75);
    private static final LoggedTunableNumber actuatorRate =
            new LoggedTunableNumber("Shooter/actuatorRate", 0.01);
    private static final LoggedTunableNumber zeroingVoltage =
            new LoggedTunableNumber("Shooter/zeroingVoltage", 1.0);
    private static final LoggedTunableNumber yawTrackingP =
            new LoggedTunableNumber("Shooter/yawTrackingP", 0.5);
    private static final LoggedTunableNumber yawTrackingDeadband =
            new LoggedTunableNumber("Shooter/yawTrackingDeadband", 0.5);
    static final LoggedTunableNumber yawMotorRotationsPerTurretRotation =
            new LoggedTunableNumber("Shooter/yawMotorRotationsPerTurretRotation", 46.0);
    private static final LoggedTunableNumber yawZeroOffsetDeg =
            new LoggedTunableNumber("Shooter/yawZeroOffsetDeg", 90.0);

    private double targetActuatorPosition = 0.5;
    private boolean wasZeroed = false;
    private boolean yawCommandedThisCycle = false;

    public Shooter(ShooterIO io, Supplier<Pose2d> robotPose) {
        this.io = io;
        this.inputs = new ShooterIOInputsAutoLogged();
        this.robotPose = robotPose;
    }

    public void periodic() {

        double startTime = Timer.getFPGATimestamp();
        io.updateInputs(inputs);

        checkForPIDUpdates();

        // Drive turret slowly counterclockwise until limit switch zeros the encoder
        if (!inputs.yawEncoderZeroed) {
            io.setTurretYawMotorVoltage(zeroingVoltage.get());
        } else if (!wasZeroed) {
            io.setTurretYawMotorVoltage(0);
            wasZeroed = true;
        } else if (!yawCommandedThisCycle) {
            // No command was sent this cycle (trigger released) — enforce limits
            io.setTurretYawMotorVoltage(0);
        }
        yawCommandedThisCycle = false;

        if (aimingAtHub) aimAtHub();

        Logger.processInputs("shooter", inputs);
        Logger.recordOutput("Shooter/targetActuatorPosition", targetActuatorPosition);
        Logger.recordOutput("Shooter/yawEncoderZeroed", inputs.yawEncoderZeroed);
        Logger.recordOutput("Shooter/aimingAtHub", aimingAtHub);
        Logger.recordOutput("Shooter/yawZeroed", inputs.yawEncoderZeroed);
        Logger.recordOutput("PeriodicTime/Shooter", (Timer.getFPGATimestamp() - startTime) * 1000);
    }

    public Command lowerForTrench() {
        aimingAtHub = false;
        return Commands.runOnce(
                () -> io.setTurretPitchPosition(shooterTrenchPitchPosition.get()), this);
    }

    public Command startShootingAtHub() {
        return this.runOnce(
                () -> io.setTurretTargetFiringVelocity(targetFlywheelRadsPerSecond.get()));
    }

    public Command stopShootingAtHub() {
        return this.runOnce(() -> io.setTurretTargetFiringVelocity(0));
    }

    public Command startAimingAtHub() {
        return Commands.runOnce(
                () -> {
                    aimingAtHub = true;
                },
                this);
    }

    public Command stopAimingAtHub() {
        return Commands.runOnce(
                () -> {
                    aimingAtHub = false;
                },
                this);
    }

    private void checkForPIDUpdates() {
        if (firingMotorP.hasChanged(this.hashCode()) || firingMotorD.hasChanged(this.hashCode())) {
            io.updatePIDFiringMotors(firingMotorP.get(), firingMotorD.get());
        }

        if (yawMotorP.hasChanged(this.hashCode()) || yawMotorD.hasChanged(this.hashCode())) {
            io.updatePIDYawMotor(yawMotorP.get(), yawMotorD.get());
        }
    }

    private void aimAtHub() {
        // Get hub position, flipped for alliance
        Translation3d hubTranslation =
                !AllianceFlipUtil.shouldFlip()
                        ? FieldConstants.Hub.topCenterPoint
                        : FieldConstants.Hub.oppTopCenterPoint;

        Translation2d shooterTranslation =
                robotPose.get().getTranslation().plus(shooterTranslationOnRobot.toTranslation2d());

        // Horizontal distance from shooter to hub
        double baseDistance = shooterTranslation.getDistance(hubTranslation.toTranslation2d());

        // Muzzle speed from fixed target flywheel speed
        double muzzleSpeed = targetFlywheelRadsPerSecond.get() * flywheelRadiusMeters;

        double horizDist = baseDistance;
        double vertDist = hubTranslation.getZ() - shooterTranslationOnRobot.getZ();

        double targetPitch = solveForPitch(muzzleSpeed, horizDist, vertDist);

        if (!Double.isNaN(targetPitch)) {
            /*
            // Second pass: re-solve with barrel correction using first-pass pitch
            // currently commented out
            horizDist = baseDistance + lengthOfShooter * Math.cos(targetPitch);
            vertDist =
                    hubTranslation.getZ()
                            - shooterTranslationOnRobot.getZ()
                            - lengthOfShooter * Math.sin(targetPitch);
            targetPitch = solveForPitch(muzzleSpeed, horizDist, vertDist); */

            io.setActuatorPosition(targetPitch);
        } else {
            System.err.println("Shooter/aimingError: Cannot solve for pitch with given parameters");
        }

        Rotation2d bearing = hubTranslation.toTranslation2d().minus(shooterTranslation).getAngle();

        // Convert to body-relative angle (how far the turret needs to rotate from robot forward)
        Rotation2d robotRelativeAngle = bearing.minus(robotPose.get().getRotation());

        // Convert body-relative angle to target motor rotations
        double gearRatio = yawMotorRotationsPerTurretRotation.get();
        double targetMotorRotations =
                robotRelativeAngle
                                .plus(Rotation2d.fromDegrees(yawZeroOffsetDeg.get()))
                                .getRotations()
                        * gearRatio;

        // Clamp target to soft limits
        targetMotorRotations =
                Math.max(
                        yawMinRotations.get(),
                        Math.min(yawMaxRotations.get(), targetMotorRotations));

        // P control on motor position error
        double positionError = targetMotorRotations - inputs.shooterPositionYaw;
        Logger.recordOutput("Shooter/yawTargetMotorRot", targetMotorRotations);
        Logger.recordOutput("Shooter/yawPositionError", positionError);

        if (Math.abs(positionError) > yawTrackingDeadband.get()) {
            double voltage = positionError * yawTrackingP.get();
            voltage = MathUtil.clamp(voltage, -maxYawVoltage.get(), maxYawVoltage.get());
            io.setTurretYawMotorVoltage(voltage);
            yawCommandedThisCycle = true;
        }

        // Command flywheel after pitch is set so angle is correct before motor spins up
        io.setTurretTargetFiringVelocity(targetFlywheelRadsPerSecond.get());
    }

    private double solveForPitch(double muzzleSpeed, double horizDist, double vertDist) {
        double g = FieldConstants.PhysicalConstants.GRAVITY;

        if (muzzleSpeed <= 0 || horizDist <= 0) return Double.NaN;
        double speedSq = muzzleSpeed * muzzleSpeed;
        double discriminant =
                speedSq * speedSq - g * (g * horizDist * horizDist + 2 * vertDist * speedSq);
        if (discriminant < 0) return Double.NaN;
        double sqrtDisc = Math.sqrt(discriminant);
        double thetaHigh = Math.atan((speedSq + sqrtDisc) / (g * horizDist));
        double thetaLow = Math.atan((speedSq - sqrtDisc) / (g * horizDist));
        return Math.max(thetaHigh, thetaLow); // higher arc
    }

    public Command raiseShooter() {
        return Commands.run(
                () -> {
                    targetActuatorPosition =
                            Math.min(1.0, targetActuatorPosition + actuatorRate.get());
                    io.setActuatorPosition(targetActuatorPosition);
                });
    }

    public Command lowerShooter() {
        return Commands.run(
                () -> {
                    targetActuatorPosition =
                            Math.max(0.0, targetActuatorPosition - actuatorRate.get());
                    io.setActuatorPosition(targetActuatorPosition);
                });
    }

    public Command runTurretYaw(DoubleSupplier input) {
        return Commands.run(
                () -> {
                    yawCommandedThisCycle = true;
                    double voltage = input.getAsDouble() * maxYawVoltage.get();
                    io.setTurretYawMotorVoltage(voltage);
                });
    }
}
