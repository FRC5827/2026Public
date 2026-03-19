package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants;
import frc.robot.FieldConstants;

import java.util.Optional;
import java.util.function.Supplier;

public class FieldUtil {
    private FieldUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static boolean isInEitherAllianceZone(Supplier<Pose2d> poseSupplier) {
        Pose2d pose = poseSupplier.get();
        double x = pose.getX();
        return x < FieldConstants.LinesVertical.allianceZone
                || x > FieldConstants.LinesVertical.oppAllianceZone;
    }

    public static boolean isInCurrentAllianceZone(
            Supplier<Pose2d> poseSupplier, Optional<Alliance> alliance) {
        Pose2d pose = poseSupplier.get();
        double x = pose.getX();
        if (alliance.isEmpty()) {
            return false;
        }
        if (alliance.get() == Alliance.Blue) {
            return x < FieldConstants.LinesVertical.allianceZone + Constants.ROBOT_LENGTH;
        } else {
            return x > FieldConstants.LinesVertical.oppAllianceZone - Constants.ROBOT_LENGTH;
        }
    }

    /**
     * Determines if the robot is aligned with the trench opening. This checks if the entire robot
     * is within the left or right trench opening bounds, not just the center point. Also excludes
     * corner areas (near the driver stations) from triggering rumble.
     *
     * @param robotPoseSupplier the robot's current pose supplier
     * @return true if the entire robot is aligned with a trench opening and not near driver
     *     stations
     */
    public static boolean isTrenchAligned(Supplier<Pose2d> robotPoseSupplier) {
        Pose2d robotPose = robotPoseSupplier.get();
        double robotY = robotPose.getY();
        double robotX = robotPose.getX();
        // Robot bumper to bumper length (Y dimension) - 0.826m is width, kept for reference
        double halfLength = Constants.ROBOT_LENGTH / 2.0;

        // Calculate the bounds of the entire robot
        double robotYMin = robotY - halfLength;
        double robotYMax = robotY + halfLength;

        // Check if entire robot fits within left trench opening
        boolean isAlignedWithLeftTrench =
                robotYMin >= FieldConstants.LinesHorizontal.leftTrenchOpenEnd
                        && robotYMax <= FieldConstants.LinesHorizontal.leftTrenchOpenStart;

        // Check if entire robot fits within right trench opening
        boolean isAlignedWithRightTrench =
                robotYMin >= FieldConstants.LinesHorizontal.rightTrenchOpenEnd
                        && robotYMax <= FieldConstants.LinesHorizontal.rightTrenchOpenStart;

        // Exclude corners near driver stations - don't rumble near the ends of the field
        // Field length is approximately 16.54 meters, so corners are at X near 0 and X near 16.54
        double cornerExclusionDistance = 1.2; // meters from each end
        boolean isNearDriverStations =
                robotX < cornerExclusionDistance
                        || robotX > (FieldConstants.fieldLength - cornerExclusionDistance);

        boolean isTrenchAligned =
                (isAlignedWithLeftTrench || isAlignedWithRightTrench) && !isNearDriverStations;

        return isTrenchAligned;
    }
}
