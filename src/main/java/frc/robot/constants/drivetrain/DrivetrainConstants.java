package frc.robot.constants.drivetrain;

import java.io.UncheckedIOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants 
{
    /** Meters per second */
    public static final double MAX_SPEED = 6;

    /** Radians per second */
    public static final double MAX_ANGULAR_RATE = Math.PI;

        public static final AprilTagFieldLayout TAG_LAYOUT;
        
        static {
            try {
                TAG_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
            } catch (UncheckedIOException e) {
                e.printStackTrace();
                throw new RuntimeException(e);
            }
        }

        public static final double AMBIGUITY_THRESHOLD = 0.25;
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(Units.inchesToMeters(6.676595), 0.0, Units.inchesToMeters(27.63304)), new Rotation3d(0, -45, 0));
}
