package frc.robot.constants.elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.constants.Hardware;

public class ElevatorConstants
{
    // Position Values
    public static final double ELEVATOR_BOTTOM_POSITION = 0.04; // the bottom position of the elevator
    public static final double ELEVATOR_TOP_POSITION = 0.74; // the top position of the elevator
    public static final double ELEVATOR_BOTTOM_CLEARANCE_POSITION = 0.07; // the position at which the shooter can begin to clear the bar
    public static final double ELEVATOR_MAX_FORWARD_POS = .78; // the top limit position of the elevator
    public static final double ELEVATAOR_MAX_REVERSE_POS = 0; // the bottom limit position of the elevator
    public static final double ELEVATOR_BAR_POSITION = 0.453; // the position of the bar on the elevator
    public static final double ELEVATOR_INTAKE_CLEAR_POSITION = 0.2; // the position at which the intake needs to move to allow the elevator to clear
    public static final double ELEVATOR_DOWN_CHECK_POSITION = 0.05; // the position at which to check if the shooter is flat yet when moving the elevator down
    public static final double ELEVATOR_UP_CHECK_POSITION = 0.16; // the position at which to check if the shooter will clear the bar when going up
    public static final double ELEVATOR_BUMPER_SHOT_POSITION = 0.127;
    public static final double ELEVATOR_AMP_SHOT_POSITION = 0.097412;
    public static final double ELEVATOR_MANUAL_SHOT_POSITION = 0.127;
    public static final double ELEVATOR_CLIMB_POSITION = ELEVATOR_BAR_POSITION + 0.05;
    public static final double ELEVATOR_PURGE_POSITION = 0.125;
    public static final double ELEVATOR_CLIMB_SAFE_DOWN = 0.215;

    public static final double ELEVATOR_FEED_FORWARD = 0.26; 

    /** Gains for the elevator motors */
    public static final Slot0Configs ELEVATOR_GAINS = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withKG(0).withGravityType(GravityTypeValue.Elevator_Static);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(5)
        .withMotionMagicAcceleration(0.3);

    /** Software limits for the elevator */
    public static final SoftwareLimitSwitchConfigs SOFT_LIMIT_CONFIGS = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ELEVATOR_MAX_FORWARD_POS)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ELEVATAOR_MAX_REVERSE_POS);

    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(Hardware.ELEVATOR_CANCODER_ID)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withRotorToSensorRatio(1)
        .withSensorToMechanismRatio(1);

    public static final MotorOutputConfigs BRAKE_CONFIG = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    public static final MagnetSensorConfigs MAGNET_SENSOR_CONFIGS = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-0.49)
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    
    public static final TalonFXConfiguration ELEVATOR_MOTOR_CONFIG = new TalonFXConfiguration()
        .withSlot0(ELEVATOR_GAINS)
        .withMotionMagic(MOTION_MAGIC_CONFIGS)
        .withSoftwareLimitSwitch(SOFT_LIMIT_CONFIGS)
        .withFeedback(FEEDBACK_CONFIGS)
        .withMotorOutput(BRAKE_CONFIG);

    public static final CANcoderConfiguration ELEVATOR_CANCODER_CONFIGURATION = new CANcoderConfiguration()
        .withMagnetSensor(MAGNET_SENSOR_CONFIGS);

    // Misc
    public static final double ELEVATOR_POSITION_TOLERANCE = 0.004;
}