// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String cameraFRName = "ROLEY-FR_OV9281";
    public static String cameraFLName = "ROLEY-FL_OV9281";
    public static String cameraBRName = "ROLEY-BR_OV9281";
    public static String cameraBLName = "ROLEY-BL_OV9281";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    // Cameras in 21.5" square (10.75" = 0.273m from center), 9" high (0.229m)
    // Pitch: 20° up, Yaw: 20° outward toward corners
    public static Transform3d robotToCameraFR =
            new Transform3d(
                    0.273,
                    -0.273,
                    0.229,
                    new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(-20)));
    public static Transform3d robotToCameraFL =
            new Transform3d(
                    0.273,
                    0.273,
                    0.229,
                    new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(20)));
    public static Transform3d robotToCameraBR =
            new Transform3d(
                    -0.273,
                    -0.273,
                    0.229,
                    new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(-160)));
    public static Transform3d robotToCameraBL =
            new Transform3d(
                    -0.273,
                    0.273,
                    0.229,
                    new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(160)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[] {
                1.0, // Front Right
                1.0, // Front Left
                1.0, // Back Right
                1.0 // Back Left
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
}
