// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants
{
    // Camera Stuff
    public static final double AMBIGUITY_THRESHOLD = 0.2;

    public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    public static final PoseStrategy FALLBACK_POSE_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;

    // Cameras
    public static final ArrayList<Pair<String, Transform3d>> CAMERAS = new ArrayList<Pair<String, Transform3d>>
    (
        List.of
        (
            // CAM RIGHT
            new Pair<String, Transform3d>
            (
                "CameraRight1",
                new Transform3d
                (
                    new Translation3d(Units.inchesToMeters(-8.32), Units.inchesToMeters(-12.745-1), Units.inchesToMeters(25.104+1)), 
                    new Rotation3d(Rotation2d.fromDegrees(0).getRadians(), Rotation2d.fromDegrees(-48.004).getRadians(), Rotation2d.fromDegrees(-90).getRadians())
                )
            ),
            // CAM LEFT
            new Pair<String, Transform3d>
            (
                "CameraLeft1",
                new Transform3d
                (
                    new Translation3d(Units.inchesToMeters(-8.32), Units.inchesToMeters(-12.745-1), Units.inchesToMeters(25.104+1)), 
                    new Rotation3d(Rotation2d.fromDegrees(0).getRadians(), Rotation2d.fromDegrees(-48.004).getRadians(), Rotation2d.fromDegrees(-90).getRadians())
                )
            )
        )
    );

    // Field Stuff
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static class FIELD_POSES
    {
        public static final Pose2d RED_SPEAKER = FIELD_LAYOUT.getTagPose(4).get().toPose2d();
        public static final Pose2d BLUE_SPEAKER = FIELD_LAYOUT.getTagPose(7).get().toPose2d();

        public static final Pose2d RED_AMP = FIELD_LAYOUT.getTagPose(5).get().toPose2d();
        public static final Pose2d BLUE_AMP = FIELD_LAYOUT.getTagPose(6).get().toPose2d();
    }
}
