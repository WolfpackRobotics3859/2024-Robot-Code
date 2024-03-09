package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
        public static final double ELEVATOR_POSITION = 0.11;
    }

    public static final class INTAKING
    {
        public static final double INTAKE_ROLLER_VOLTAGE = -6;
        public static final double INTAKE_WRIST_POSITION = 0.23;
        public static final double SHOOTER_ROLLER_1_VELOCITY = -30;
        public static final double SHOOTER_ROLLER_2_VELOCITY = -30;
        public static final double SHOOTER_FEEDER_VOLTAGE = -3;
        public static final double SHOOTER_WRIST_ANGLE = 0.5;
        public static final double ELEVATOR_POSITION = 0.11;
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

    public static final class POSES
    {
        public static final Pose3d BLUE_AMP_3D_POSE = DriveConstants.TAG_LAYOUT.getTagPose(5).get();
        public static final Pose2d BLUE_AMP_POSE = new Pose2d(BLUE_AMP_3D_POSE.getX(), BLUE_AMP_3D_POSE.getY(), BLUE_AMP_3D_POSE.getRotation().toRotation2d());

        public static final Pose3d RED_AMP_3D_POSE = DriveConstants.TAG_LAYOUT.getTagPose(4).get();
        public static final Pose2d RED_AMP_POSE = new Pose2d(RED_AMP_3D_POSE.getX(), RED_AMP_3D_POSE.getY(), RED_AMP_3D_POSE.getRotation().toRotation2d());
    }
}
