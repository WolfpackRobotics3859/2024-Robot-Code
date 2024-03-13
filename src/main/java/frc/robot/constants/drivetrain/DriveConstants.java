package frc.robot.constants.drivetrain;

import java.io.UncheckedIOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class DriveConstants 
{
    /** Meters per second */
    public static final double MAX_SPEED = 6;

    /** Radians per second */
    public static final double MAX_ANGULAR_RATE = Math.PI;

    public static final AprilTagFieldLayout TAG_LAYOUT;
        
        static 
        {
            try 
            {
                TAG_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
            } 
            catch (UncheckedIOException e) 
            {
                e.printStackTrace();
                throw new RuntimeException(e);
            }
        }

        public static final double AMBIGUITY_THRESHOLD = 0.25;

        public static final class CAMERA_POSITIONS
        {
            public static final Transform3d RIGHT_1 = 
                new Transform3d(new Translation3d(Units.inchesToMeters(12.142), 
                                                  Units.inchesToMeters(0.0372), 
                                                  Units.inchesToMeters(16.157)), 
                                                  new Rotation3d(0, Rotation2d.fromDegrees(-10).getDegrees(), 0));

            public static final Transform3d LEFT_1 = 
                new Transform3d(new Translation3d(Units.inchesToMeters(12.142), 
                                                  Units.inchesToMeters(0.0372), 
                                                  Units.inchesToMeters(16.157)), 
                                                  new Rotation3d(0, Rotation2d.fromDegrees(-10).getDegrees(), 0));
                
            public static final Transform3d REAR_1 = 
                new Transform3d(new Translation3d(Units.inchesToMeters(12.142), 
                                                  Units.inchesToMeters(0.0372), 
                                                  Units.inchesToMeters(16.157)), 
                                                  new Rotation3d(0, Rotation2d.fromDegrees(-10).getDegrees(), 0));
        }
}
