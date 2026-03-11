package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.Supplier;

/**
 * Stateful aiming calculator for shooting trajectories.
 *
 * <p>Set the active target with {@link #setTarget} and clear it with {@link #clearTarget}. Results
 * are available via the getter methods and are also logged automatically in {@link #periodic}.
 *
 * <p>Lives in the shooter package to access package-private tunables on {@link Turret} and {@link
 * Shooter} directly.
 */
public class Targeting extends SubsystemBase {
    // Translation of shooter from robot center (m)
    static final Transform3d SHOOTER_TRANSLATION_ON_ROBOT =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(6.5),
                            Units.inchesToMeters(-4.5),
                            Units.inchesToMeters(16)),
                    Rotation3d.kZero);
    // Multiplier to account for lack of acceleration under hood
    static final LoggedTunableNumber shooterMultiplier =
            new LoggedTunableNumber("Targeting/Shooter Multiplier", 2.4);

    static final LoggedTunableNumber manualTargetPitchAngleRad =
            new LoggedTunableNumber(
                    "Targeting/Manual/Target Pitch Angle Radians", Units.degreesToRadians(45));
    static final LoggedTunableNumber manualTargetDistanceMeters =
            new LoggedTunableNumber("Targeting/Manual/Target Distance Meters", 1.0);

    static final LoggedTunableNumber distanceFromCornerToCornerShot =
            new LoggedTunableNumber("Targeting/Distance From Corner To Corner Shot", 0.5);

    public static LoggedDashboardChooser<Boolean> overrideTimeRestrictions;
    public static LoggedDashboardChooser<Boolean>
            targetCornerIsLeftCorner; // Depot corner, Outpost Corner

    private final Supplier<Pose2d> robotPoseSupplier;

    // --- Target (null = no active target) ---
    private Translation3d targetTranslation = null;
    private Translation2d targetClearance = null;

    // --- Pose tracking for yaw velocity estimate ---
    private Pose2d lastPose;
    private double lastPoseTimestamp;
    private final LinearFilter xFilter, yFilter, rFilter;

    // --- Computed state (updated each periodic()) ---
    private double pitchAngle = 0.0;
    private double shooterVelocity = 0.0;
    private double yawPosition = 0.0;
    private double yawVelocity = 0.0;
    private double[] trajectoryCoefficients = null; // can be null
    private boolean canAimAtTarget = false;

    public Targeting(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.lastPose = robotPoseSupplier.get();
        this.lastPoseTimestamp = Timer.getFPGATimestamp();
        this.xFilter = LinearFilter.movingAverage(10);
        this.yFilter = LinearFilter.movingAverage(10);
        this.rFilter = LinearFilter.movingAverage(10);

        overrideTimeRestrictions =
                new LoggedDashboardChooser<>("Override Time Restrictions", new SendableChooser<>());
        overrideTimeRestrictions.addDefaultOption("False", false);
        overrideTimeRestrictions.addOption("True", true);

        targetCornerIsLeftCorner =
                new LoggedDashboardChooser<>("Chosen Corner For Passing", new SendableChooser<>());
        targetCornerIsLeftCorner.addDefaultOption("Left Corner", true);
        targetCornerIsLeftCorner.addOption("Right Corner", false);
    }

    /**
     * Sets the active aiming target. Call this before enabling aiming commands.
     *
     * @param targetTranslation 3D field-relative target location
     * @param targetClearance Extra clearance required (x = horizontal, y = vertical)
     */
    public void setTarget(Translation3d targetTranslation, Translation2d targetClearance) {
        this.targetTranslation = targetTranslation;
        this.targetClearance = targetClearance;
    }

    public void setTargetManual() {
        this.pitchAngle = manualTargetPitchAngleRad.get();
        this.shooterVelocity =
                solveForVelocityWithAngle(pitchAngle, manualTargetDistanceMeters.get(), 0);
        this.yawPosition = 0.0;
        this.yawVelocity = 0.0;
        this.trajectoryCoefficients = null;
        this.canAimAtTarget = true;
    }

    public void lowerForTrench() {
        this.pitchAngle = Turret.pitchMinAngleRad.get();
        this.shooterVelocity = 0.0;
        this.yawVelocity = 0.0;
        this.trajectoryCoefficients = null;
        this.canAimAtTarget = false;
    }

    /** Clears the active target; aiming calculations stop until a new target is set. */
    public void clearTarget() {
        this.targetTranslation = null;
        this.targetClearance = null;
        this.canAimAtTarget = false;
        this.shooterVelocity = 0.0;
    }

    public void aimAtCorner() {
        double oneOverRoot2 = 1 / Math.sqrt(2); // For 45 degree angle to corner
        Translation3d targetTranslation =
                new Translation3d(
                        distanceFromCornerToCornerShot.get() * oneOverRoot2,
                        !targetCornerIsLeftCorner.get()
                                ? distanceFromCornerToCornerShot.get() * oneOverRoot2
                                : FieldConstants.fieldWidth
                                        - distanceFromCornerToCornerShot.get() * oneOverRoot2,
                        0);

        double distanceFromTrench;
        double trenchHeight;
        Pose2d robotPose = robotPoseSupplier.get();
        if (targetCornerIsLeftCorner.get()) {
            distanceFromTrench =
                    robotPose
                            .getTranslation()
                            .getDistance(
                                    new Translation2d(
                                            FieldConstants.LinesVertical.hubCenter,
                                            FieldConstants.fieldWidth
                                                    - FieldConstants.LeftTrench.width));
            trenchHeight = FieldConstants.LeftTrench.height;
        } else {
            distanceFromTrench =
                    robotPose
                            .getTranslation()
                            .getDistance(
                                    new Translation2d(
                                            FieldConstants.LinesVertical.hubCenter,
                                            FieldConstants.RightTrench.width));
            trenchHeight = FieldConstants.RightTrench.height;
        }

        targetTranslation = AllianceFlipUtil.apply(targetTranslation);

        setTarget(
                targetTranslation,
                new Translation2d(
                        distanceFromTrench,
                        trenchHeight + Units.inchesToMeters(5.91))); // atleast 0.5 balls above
    }

    /** Returns {@code true} if a target is currently set. */
    public boolean hasTarget() {
        return targetTranslation != null && targetClearance != null;
    }

    /**
     * Recomputes the aiming solution from the current robot pose and the active target, then logs
     * all results. No-ops if no target is set.
     *
     * <p>Called automatically every robot loop by the WPILib command scheduler
     */
    @Override
    public void periodic() {
        Pose2d robotPose = robotPoseSupplier.get();
        double dt = (Timer.getFPGATimestamp() - lastPoseTimestamp);
        Transform2d poseVelocity = robotPose.minus(lastPose).div(dt);
        double filteredX = xFilter.calculate(poseVelocity.getX());
        double filteredY = yFilter.calculate(poseVelocity.getY());
        double filteredR = rFilter.calculate(poseVelocity.getRotation().getRotations());
        Transform2d filteredTransform =
                new Transform2d(filteredX, filteredY, Rotation2d.fromRotations(filteredR));
        if (hasTarget()) {
            update(robotPose, filteredTransform);
        } else {
            lowerForTrench();
        }

        // Always log so AdvantageKit captures cleared state too
        Logger.recordOutput("Aim/Has Target", hasTarget());
        Logger.recordOutput("Aim/Can Aim At Target", canAimAtTarget);
        Logger.recordOutput("Aim/Pitch Angle", pitchAngle);
        Logger.recordOutput("Aim/Shooter Velocity", shooterVelocity);
        Logger.recordOutput("Aim/Yaw Position", yawPosition);
        Logger.recordOutput("Aim/Yaw Velocity", yawVelocity);
        Logger.recordOutput("Aim/Trajectory Coefficients", trajectoryCoefficients);

        // Advance pose tracking
        lastPose = robotPose;
        lastPoseTimestamp = Timer.getFPGATimestamp();
    }

    // --- Getters ---

    public Translation3d getTargetTranslation() {
        return targetTranslation;
    }

    public double getPitchAngle() {
        return pitchAngle;
    }

    public double getShooterVelocity() {
        return shooterVelocity;
    }

    public double getYawPosition() {
        return yawPosition;
    }

    public double getYawVelocity() {
        return yawVelocity;
    }

    public boolean canAimAtTarget() {
        return canAimAtTarget;
    }

    public double getAirTimeToTarget() {
        if (trajectoryCoefficients == null || targetTranslation == null) {
            return 0;
        }
        // Time to reach target is time to reach horizontal distance at horizontal velocity
        double horizontalVelocity = shooterVelocity * Math.cos(pitchAngle);
        if (horizontalVelocity <= 0) {
            return 0;
        }
        double horizontalDistance = targetTranslation.toTranslation2d().getNorm();
        return horizontalDistance / horizontalVelocity;
    }

    // --- Internal computation ---

    private void update(Pose2d robotPose, Transform2d filteredVelocity) {

        Translation3d shooterTranslation =
                new Pose3d(robotPose).transformBy(SHOOTER_TRANSLATION_ON_ROBOT).getTranslation();

        Translation3d shooterToTargetTranslation = targetTranslation.minus(shooterTranslation);

        double[] pitchAndVelocity =
                calculatePitchAndVelocityWithTranslations(shooterToTargetTranslation);

        // if we get a valid pitch and velocity, compensate for robot velocity and recalculate
        // using a simple linear approximation
        if (pitchAndVelocity[1] != 0) {
            // after we get computed values check how long it takes to score
            double timeToScore =
                    shooterToTargetTranslation.toTranslation2d().getNorm()
                            / (pitchAndVelocity[1] * Math.cos(pitchAndVelocity[0]));

            // combined with robot velocity get the new "effective shooter pose"
            shooterTranslation =
                    new Pose3d(robotPose.transformBy(filteredVelocity.times(timeToScore)))
                            .transformBy(SHOOTER_TRANSLATION_ON_ROBOT)
                            .getTranslation();
            shooterToTargetTranslation = targetTranslation.minus(shooterTranslation);

            // recalculate pitch and velocity with new shooter pose
            pitchAndVelocity =
                    calculatePitchAndVelocityWithTranslations(shooterToTargetTranslation);
        }

        double computedPitch = pitchAndVelocity[0];
        double computedVelocity = pitchAndVelocity[1] * shooterMultiplier.get();

        double computedYawPosition =
                calculateYawPosition(robotPose, shooterToTargetTranslation.toTranslation2d());

        canAimAtTarget =
                computedYawPosition >= Turret.yawMinRotations.get()
                        && computedYawPosition <= Turret.yawMaxRotations.get()
                        && computedVelocity != 0;

        double computedYawVelocity = 0.0;
        // check if yaw in range before calculating velocity to avoid noise when target is out of
        // range
        if (computedYawPosition >= Turret.yawMinRotations.get()
                && computedYawPosition <= Turret.yawMaxRotations.get()) {
            computedYawVelocity = -filteredVelocity.getRotation().getRotations();
        }

        pitchAngle = computedPitch;
        shooterVelocity = computedVelocity;
        yawPosition = computedYawPosition;
        yawVelocity = computedYawVelocity;
    }

    private double[] calculatePitchAndVelocityWithTranslations(
            Translation3d shooterToTargetTranslation) {
        double dist = shooterToTargetTranslation.toTranslation2d().getNorm();
        double shooterToTargetVertical = shooterToTargetTranslation.getZ();

        trajectoryCoefficients =
                solveQuadraticSystem(
                        0,
                        0,
                        dist - targetClearance.getX(),
                        shooterToTargetVertical + targetClearance.getY(),
                        dist,
                        shooterToTargetVertical);

        double computedPitch = Turret.pitchMaxAngleRad.get();
        double computedVelocity = 0.0;

        if (trajectoryCoefficients != null) {
            double[] values =
                    solveForPitchAndVelocity(
                            trajectoryCoefficients[0],
                            trajectoryCoefficients[1],
                            trajectoryCoefficients[2]);
            computedPitch = values[0];

            if (computedPitch < Turret.pitchMinAngleRad.get()) {
                computedPitch = Turret.pitchMinAngleRad.get();
                computedVelocity =
                        solveForVelocityWithAngle(computedPitch, dist, shooterToTargetVertical);
            } else if (computedPitch <= Turret.pitchMaxAngleRad.get()) {
                computedVelocity = values[1];
            } else {
                computedPitch = Turret.pitchMaxAngleRad.get();
                computedVelocity = 0;
            }

            computedVelocity = Math.max(computedVelocity, 0);
        }
        return new double[] {computedPitch, computedVelocity};
    }

    // --- Static math helpers ---

    /**
     * Calculates the yaw position needed to aim at a target.
     *
     * @param robotPose Current robot pose
     * @param shooterToTargetTranslation 2D translation from shooter to target
     * @return Yaw position in rotations
     */
    public static double calculateYawPosition(
            Pose2d robotPose, Translation2d shooterToTargetTranslation) {
        Rotation2d targetAngle = shooterToTargetTranslation.getAngle();
        Rotation2d calculatedYawPosition = targetAngle.minus(robotPose.getRotation());
        return calculatedYawPosition.getRotations();
    }

    /**
     * Solves a system of three quadratic equations to find trajectory coefficients.
     *
     * @return Array of coefficients [a, b, c] for y = ax² + bx + c, or null if invalid
     */
    public static double[] solveQuadraticSystem(
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

            // Sanity checks: d²y/dx² should be negative, dy/dx should be positive at 0
            if (vec[0] >= 0 || vec[1] <= 0) {
                return null;
            }
            return vec;
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Calculates pitch angle and velocity from quadratic trajectory coefficients.
     *
     * @param a, b, c Coefficients of quadratic equation y = ax² + bx + c
     * @return Array containing [pitch angle in radians, velocity in m/s]
     */
    public static double[] solveForPitchAndVelocity(double a, double b, double c) {
        if (c != 0) {
            return new double[] {0, 0};
        }

        double curvePeakY = -(b * b) / (4 * a);
        double verticalVelocity =
                Math.sqrt(2 * curvePeakY * FieldConstants.PhysicalConstants.GRAVITY);
        double horizontalVelocity = verticalVelocity / b;

        return new double[] {
            Math.atan2(verticalVelocity, horizontalVelocity),
            Math.hypot(horizontalVelocity, verticalVelocity)
        };
    }

    /**
     * Calculates required launch velocity for a fixed angle, distance, and height.
     *
     * @param angle Launch angle in radians
     * @param horizontalDistance Horizontal distance to target
     * @param verticalDistance Vertical distance to target
     * @return Required velocity in m/s
     */
    public static double solveForVelocityWithAngle(
            double angle, double horizontalDistance, double verticalDistance) {
        double b = Math.tan(angle);
        double a =
                (verticalDistance - b * horizontalDistance)
                        / (horizontalDistance * horizontalDistance);
        return solveForPitchAndVelocity(a, b, 0)[1];
    }
}
