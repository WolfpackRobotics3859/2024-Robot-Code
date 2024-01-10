// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.io.UncheckedIOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final AprilTagFieldLayout TAG_LAYOUT;
    static {
        try {
            TAG_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        } catch (UncheckedIOException e) {
             e.printStackTrace();
            throw new RuntimeException(e);
        }
    }
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(Units.inchesToMeters(8.5), 0.0, Units.inchesToMeters(33.5)), new Rotation3d(0, 15, 0)); //8.5 inches back from center, 33.5 inches up from center, facing forward and a 15 deg up;
}
