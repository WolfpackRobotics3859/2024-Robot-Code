package frc.robot.constants.drivetrain;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;

public class DriveConstants 
{
    // Speed Limits
    public static final double MAX_SPEED = 6;
    public static final double MAX_ANGULAR_RATE = Math.PI;

    // Turn to angle PID
    public static final double TURN_TO_ANGLE_P = 10;
    public static final double TURN_TO_ANGLE_I = 0.001;
    public static final double TURN_TO_ANGLE_D = 0;

    // Turn to angle tolerance (radians)
    public static final double TURN_TO_ANGLE_TOLERANCE = Math.PI*0.1;

    // Operator Perspective
    public final static Rotation2d BLUE_OPERATOR_FORWARD_PERSPECTIVE = Rotation2d.fromDegrees(0);
    public final static Rotation2d RED_OPERATOR_FORWARD_PERSPECTIVE = Rotation2d.fromDegrees(180);  
    
    public static final Map<Double, String> PERSPECTIVE_MAP = Map.of
    (
    0.0, "Blue",
    180.0, "Red"
    );
}

