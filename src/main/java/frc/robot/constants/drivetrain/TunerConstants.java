package frc.robot.constants.drivetrain;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;

public class TunerConstants 
{
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
        .withKP(60).withKI(0).withKD(0.05)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double SLIP_CURRENT_A = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double SPEED_AT_12_VOLTS_MPS = 6;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double COUPLE_RATIO = 3.5;

    private static final double DRIVE_GEAR_RATIO = 5.6;
    private static final double STEER_GEAR_RATIO = 13.371428571428572;
    private static final double WHEEL_RADIUS_INCHES = 2;

    private static final boolean STEER_MOTOR_REVERSED = true;
    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    private static final String CANBUS_NAME = "CANivore";
    private static final int PIDGEON_ID = 0;


    // These are only used for simulation
    private static final double STEER_INERTIA = 0.00001;
    private static final double DRIVE_INERTIA = 0.001;

    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
            .withPigeon2Id(PIDGEON_ID)
            .withCANbusName(CANBUS_NAME);

    private static final SwerveModuleConstantsFactory CONSTANT_CREATOR = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withWheelRadius(WHEEL_RADIUS_INCHES)
            .withSlipCurrent(SLIP_CURRENT_A)
            .withSteerMotorGains(STEER_GAINS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSpeedAt12VoltsMps(SPEED_AT_12_VOLTS_MPS)
            .withSteerInertia(STEER_INERTIA)
            .withDriveInertia(DRIVE_INERTIA)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(COUPLE_RATIO)
            .withSteerMotorInverted(STEER_MOTOR_REVERSED);


    // Front Left
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 2;
    private static final int FRONT_LEFT_ENCODER_ID = 1;
    private static final double FRONT_LEFT_ENCODER_OFFSET = -0.129883;

    private static final double FRONT_LEFT_X_POS_INCHES = 11.75;
    private static final double FRONT_LEFT_Y_POS_INCHES = 11.75;

    // Front Right
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 7;
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 8;
    private static final int FRONT_RIGHT_ENCODER_ID = 4;
    private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.102295;

    private static final double FRONT_RIGHT_X_POS_INCHES = 11.75;
    private static final double FRONT_RIGHT_Y_POS_INCHES = -11.75;

    // Back Left
    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 3;
    private static final int BACK_LEFT_STEER_MOTOR_ID = 4;
    private static final int BACK_LEFT_ENCODER_ID = 2;
    private static final double BACK_LEFT_ENCODER_OFFSET = 0.214355;

    private static final double BACK_LEFT_X_POS_INCHES = -11.75;
    private static final double BACK_LEFT_Y_POS_INCHES = 11.75;

    // Back Right
    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 5;
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 6;
    private static final int BACK_RIGHT_ENCODER_ID = 3;
    private static final double BACK_RIGHT_ENCODER_OFFSET = 0.43017578125;

    private static final double BACK_RIGHT_X_POS_INCHES = -11.75;
    private static final double BACK_RIGHT_Y_POS_INCHES = -11.75;


    public static final SwerveModuleConstants FRONT_LEFT = CONSTANT_CREATOR.createModuleConstants(
            FRONT_LEFT_STEER_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_ENCODER_ID, FRONT_LEFT_ENCODER_OFFSET, Units.inchesToMeters(FRONT_LEFT_X_POS_INCHES), Units.inchesToMeters(FRONT_LEFT_Y_POS_INCHES), INVERT_LEFT_SIDE);
    public static final SwerveModuleConstants FRONT_RIGHT = CONSTANT_CREATOR.createModuleConstants(
            FRONT_RIGHT_STEER_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_ENCODER_ID, FRONT_RIGHT_ENCODER_OFFSET, Units.inchesToMeters(FRONT_RIGHT_X_POS_INCHES), Units.inchesToMeters(FRONT_RIGHT_Y_POS_INCHES), INVERT_RIGHT_SIDE);
    public static final SwerveModuleConstants BACK_LEFT = CONSTANT_CREATOR.createModuleConstants(
            BACK_LEFT_STEER_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_ENCODER_ID, BACK_LEFT_ENCODER_OFFSET, Units.inchesToMeters(BACK_LEFT_X_POS_INCHES), Units.inchesToMeters(BACK_LEFT_Y_POS_INCHES), INVERT_LEFT_SIDE);
    public static final SwerveModuleConstants BACK_RIGHT = CONSTANT_CREATOR.createModuleConstants(
            BACK_RIGHT_STEER_MOTOR_ID, BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_ENCODER_ID, BACK_RIGHT_ENCODER_OFFSET, Units.inchesToMeters(BACK_RIGHT_X_POS_INCHES), Units.inchesToMeters(BACK_RIGHT_Y_POS_INCHES), INVERT_RIGHT_SIDE);
}
