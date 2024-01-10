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
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(Units.inchesToMeters(8.5), 0.0, Units.inchesToMeters(33.5)), new Rotation3d(0, 15, 0)); //8.5 inches back from center, 33.5 inches up from center, facing forward and a 15 deg up, is a rough guestimate from lack of camera mount

}
