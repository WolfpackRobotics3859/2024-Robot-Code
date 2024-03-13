package frc.robot.constants;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.drivetrain.DriveConstants;

public class Positions 
{
    public static final class STOW
    {
        public static final double INTAKE_ROLLER_VOLTAGE = 0;
        public static final double INTAKE_WRIST_POSITION = 4.3;
        public static final double SHOOTER_ROLLER_1_VELOCITY = 0;
        public static final double SHOOTER_ROLLER_2_VELOCITY = 0;
        public static final double SHOOTER_FEEDER_VOLTAGE = 0;
        public static final double SHOOTER_WRIST_ANGLE = 0.5;
        public static final double ELEVATOR_POSITION = 0.0725;
    }

    public static final class INTAKING
    {
        public static final double INTAKE_ROLLER_VOLTAGE = -6;
        public static final double INTAKE_WRIST_POSITION = 0.23;
        public static final double SHOOTER_ROLLER_1_VELOCITY = -30;
        public static final double SHOOTER_ROLLER_2_VELOCITY = -30;
        public static final double SHOOTER_FEEDER_VOLTAGE = -3;
        public static final double SHOOTER_WRIST_ANGLE = 0.53;
        public static final double ELEVATOR_POSITION = 0.0725;
    }

    public static final class AMP
    {
        public static final double INTAKE_ROLLER_VOLTAGE = 0;
        public static final double INTAKE_WRIST_POSITION = 3.0;
        public static final double SHOOTER_ROLLER_1_VELOCITY = -30;
        public static final double SHOOTER_ROLLER_2_VELOCITY = -30;
        public static final double SHOOTER_FEEDER_VOLTAGE = -10;
        public static final double SHOOTER_WRIST_ANGLE = 0.53;
        public static final double ELEVATOR_POSITION = 0.84;
    }

    public static final class LOW_SHOT
    {
        public static final double INTAKE_ROLLER_VOLTAGE = 0;
        public static final double INTAKE_WRIST_POSITION = 2.7;
        public static final double SHOOTER_FEEDER_VOLTAGE = 10;
        public static final double ELEVATOR_POSITION = 0.275;
    }

    public static final class APRILTAGS
    {
        public static final Pose3d BLUE_AMP_3D_POSE = DriveConstants.TAG_LAYOUT.getTagPose(5).get();
        public static final Pose2d BLUE_AMP_POSE = new Pose2d(BLUE_AMP_3D_POSE.getX(), BLUE_AMP_3D_POSE.getY(), BLUE_AMP_3D_POSE.getRotation().toRotation2d());

        public static final Pose3d RED_AMP_3D_POSE = DriveConstants.TAG_LAYOUT.getTagPose(4).get();
        public static final Pose2d RED_AMP_POSE = new Pose2d(RED_AMP_3D_POSE.getX(), RED_AMP_3D_POSE.getY(), RED_AMP_3D_POSE.getRotation().toRotation2d());

        public static final AprilTag BLUE_SPEAKER_TAG = DriveConstants.TAG_LAYOUT.getTags().get(7);
        public static final AprilTag RED_SPEAKER_TAG = DriveConstants.TAG_LAYOUT.getTags().get(4);

        /**
         * Gets the current speaker the robot should be aimed at based on the DriverStation or FMS
         * @return The AprilTag object of the desired speaker (if none present, returns null)
         */
        public static final AprilTag getCurrentSpeakerTag()
        {
            return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
            ? BLUE_SPEAKER_TAG
            : RED_SPEAKER_TAG;
        }

        public static final Supplier<Pose2d> SPEAKER_POSE_SUPPLIER = () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
            ? BLUE_SPEAKER_TAG.pose.toPose2d()
            : RED_SPEAKER_TAG.pose.toPose2d();
    }

    public static final class LOW_BUMPER_SHOT
    {
        public static final double INTAKE_ROLLER_VOLTAGE = 0;
        public static final double INTAKE_WRIST_POSITION = 2.7;
        public static final double SHOOTER_ROLLER_1_VELOCITY = 35;
        public static final double SHOOTER_ROLLER_2_VELOCITY = 35;
        public static final double SHOOTER_FEEDER_VOLTAGE = 10;
        public static final double SHOOTER_WRIST_ANGLE = 0.635;
        public static final double ELEVATOR_POSITION = 0.275;
    }

    public static final class DEFENSE_SHOT
    {
        public static final double INTAKE_ROLLER_VOLTAGE = 0;
        public static final double INTAKE_WRIST_POSITION = 3.0;
        public static final double SHOOTER_ROLLER_1_VELOCITY = 30;
        public static final double SHOOTER_ROLLER_2_VELOCITY = 30;
        public static final double SHOOTER_FEEDER_VOLTAGE = -10;
        public static final double SHOOTER_WRIST_ANGLE = 0.53;
        public static final double ELEVATOR_POSITION = 0.83; 
    }
}
