package frc.robot.constants.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.constants.Hardware;

public class ShooterConstants
{

    // Shooter motors
    public static final double SHOOTER_MOTOR_ACCELERATION = 40;

    public static final Slot0Configs SHOOTER_1_GAINS = new Slot0Configs()
        .withKP(0.015365).withKI(0).withKD(0)
        .withKS(0.035335).withKV(0.11133).withKA(0.0098933);

    public static final Slot0Configs SHOOTER_2_GAINS = new Slot0Configs()
        .withKP(0.015365).withKI(0).withKD(0)
        .withKS(0.035335).withKV(0.11133).withKA(0.0098933);
    
    // Wrist motor
    public static final Slot0Configs WRIST_GAINS = new Slot0Configs()
        .withKP(0.03).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // Feeder motor
    public static final Slot0Configs FEEDER_GAINS = new Slot0Configs()
        .withKP(0.03).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    
    public static final double WRIST_VELOCITY = 4;
    public static final double SHOOTER_WRIST_MAX_FORWARD_ROTATION = .2;
    public static final double SHOOTER_WRIST_MAX_REVERSE_ROTATION = 0;
    public static final double SHOOTER_WRIST_FEED_FORWARD = 0;

    // Wrist configs
    public static final MotionMagicConfigs WRIST_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(10)
        .withMotionMagicAcceleration(2);

    public static final SoftwareLimitSwitchConfigs WRIST_SOFT_LIMIT_CONFIGS = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(SHOOTER_WRIST_MAX_FORWARD_ROTATION)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(SHOOTER_WRIST_MAX_REVERSE_ROTATION);

    public static final FeedbackConfigs WRIST_FEEDBACK_CONFIGS = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(Hardware.SHOOTER_WRIST_CANCODER_ID)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withRotorToSensorRatio(1)
        .withSensorToMechanismRatio(1);

    public static final MotorOutputConfigs WRIST_BRAKE_CONFIG = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    
    // Wrist CANCoder
    public static final MagnetSensorConfigs WRIST_CANCODER_MAGNET_CONFIGS = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(0.269);

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
}
