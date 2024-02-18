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
    public static final double ELEVATOR_HANDOFF_POSITION = 0;
    public static final double ELEVATOR_BOTTOM_POSITION = 0.01;
    public static final double ELEVATOR_TOP_POSITION = 0.135;
    public static final double ELEVATOR_BOTTOM_CLEARANCE_POSITION = 0.02;
    public static final double ELEVATOR_FEED_FORWARD = 0.24;
    public static final double ELEVATOR_MAX_FORWARD_POS = .15;
    public static final double ELEVATAOR_MAX_REVERSE_POS = 0;
    public static final double ELEVATOR_SHOOTING_POSITION = 0.04; // temporary
    public static final double ELEVATOR_BAR_POSITION = 0;

    /** Gains for the elevator motors */
    public static final Slot0Configs ELEVATOR_GAINS = new Slot0Configs()
        .withKP(10).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withKG(0).withGravityType(GravityTypeValue.Elevator_Static);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0.05)
        .withMotionMagicAcceleration(0.05);

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
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(0)
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
    public static final double ELEVATOR_POSITION_TOLERANCE = 0.009;
}
