package frc.robot.constants.shooter;

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

public class ShooterConstants
{
    // Position Values
    public static final double WRIST_HANDOFF_POSITION = 0;
    public static final double WRIST_CLEARANCE_POSITION = 0.95;
    public static final double WRIST_MAX_DOWN_POSITION = 0.3;
    public static final double WRIST_MAX_UP_POSIION = 1;
    public static final double WRIST_DEFAULT_POSITION = 0.51;
    public static final double WRIST_SHOOTING_POSITION = 0.65; // temporary
    public static final double WRIST_AMP_SHOOTING_POSITION = 0;

    // Wrist values
    public static final double SHOOTER_WRIST_FEED_FORWARD = 0;

    // Shooter motor configs
    public static final double SHOOTER_MOTOR_ACCELERATION = 40;

    public static final Slot0Configs SHOOTER_1_GAINS = new Slot0Configs()
        .withKP(0.010594).withKI(0).withKD(0)
        .withKS(0.06616).withKV(0.10904).withKA(0.012043);

    public static final Slot0Configs SHOOTER_2_GAINS = new Slot0Configs()
        .withKP(0.0026316).withKI(0).withKD(0)
        .withKS(0.19959).withKV(0.1108).withKA(0.015865);
    
    // Wrist motor configs
    public static final Slot0Configs WRIST_GAINS = new Slot0Configs()
        .withKP(10).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    // Feeder motor
    public static final Slot0Configs FEEDER_GAINS = new Slot0Configs()
        .withKP(0.36523).withKI(0).withKD(0)
        .withKS(0.15837).withKV(0.10739).withKA(0.0018114);

    // Wrist configs
    public static final MotionMagicConfigs WRIST_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(2)
        .withMotionMagicAcceleration(5);

    public static final SoftwareLimitSwitchConfigs WRIST_SOFT_LIMIT_CONFIGS = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(false)
        .withForwardSoftLimitThreshold(WRIST_MAX_DOWN_POSITION)
        .withReverseSoftLimitEnable(false)
        .withReverseSoftLimitThreshold(WRIST_MAX_UP_POSIION);

    public static final FeedbackConfigs WRIST_FEEDBACK_CONFIGS = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(Hardware.SHOOTER_WRIST_CANCODER_ID)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withRotorToSensorRatio(1)
        .withSensorToMechanismRatio(1);

    public static final MotorOutputConfigs WRIST_BRAKE_CONFIG = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);
    
    // Wrist CANCoder
    public static final MagnetSensorConfigs WRIST_CANCODER_MAGNET_CONFIGS = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(0)
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);

    // Compile all configs into a single config variable for ease of use
    public static final TalonFXConfiguration SHOOTER_MOTOR_1_CONFIGURATION = new TalonFXConfiguration()
        .withSlot0(SHOOTER_1_GAINS);

    public static final TalonFXConfiguration SHOOTER_MOTOR_2_CONFIGURATION = new TalonFXConfiguration()
        .withSlot0(SHOOTER_2_GAINS);

    public static final TalonFXConfiguration FEEDER_MOTOR_CONFIGURATION = new TalonFXConfiguration()
        .withSlot0(FEEDER_GAINS);

    public static final TalonFXConfiguration WRIST_MOTOR_CONFIGURATION = new TalonFXConfiguration()
        .withFeedback(WRIST_FEEDBACK_CONFIGS).withMotionMagic(WRIST_MOTION_MAGIC_CONFIGS)
        .withSlot0(WRIST_GAINS).withMotorOutput(WRIST_BRAKE_CONFIG)
        .withSoftwareLimitSwitch(WRIST_SOFT_LIMIT_CONFIGS);

    public static final CANcoderConfiguration WRIST_CANCODER_CONFIGURATION = new CANcoderConfiguration()
        .withMagnetSensor(WRIST_CANCODER_MAGNET_CONFIGS);

    // Motor Enum
    public enum MOTOR
    {
        MOTOR_1,
        MOTOR_2,
        WRIST_MOTOR,
        FEEDER_MOTOR
    }

    // Misc
    public static final double WRIST_MOVEMENT_TOLERANCE = 0.05;
    public static final double SHOOTER_VELOCITY_TOLERANCE = 1;
}
