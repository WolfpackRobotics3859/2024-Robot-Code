package frc.robot.constants.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.constants.Hardware;

public class ShooterConstants
{
    // Shooter motor 1
    public static final Slot0Configs SHOOTER_1_GAINS = new Slot0Configs()
        .withKP(0.015365).withKI(0).withKD(0)
        .withKS(0.035335).withKV(0.11133).withKA(0.0098933);

    // Shooter motor 2
    public static final Slot0Configs SHOOTER_2_GAINS = new Slot0Configs()
        .withKP(0.015365).withKI(0).withKD(0)
        .withKS(0.035335).withKV(0.11133).withKA(0.0098933);
    
    // Wrist motor
    public static final Slot0Configs WRIST_GAINS = new Slot0Configs()
        .withKP(18).withKI(0).withKD(0.1)
        .withKS(0.4).withKV(0.15).withKA(0)
        .withKG(0.05).withGravityType(GravityTypeValue.Arm_Cosine);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(10)
        .withMotionMagicAcceleration(2);

    public static final double SHOOTER_WRIST_MAX_FORWARD_POS = .2;
    public static final double SHOOTER_WRIST_MAX_REVERSE_POS = 0;
    public static final double SHOOTER_WRIST_FEED_FORWARD = 0;

    /** Software limits for the elevator */
    public static final SoftwareLimitSwitchConfigs SOFT_LIMIT_CONFIGS = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(SHOOTER_WRIST_MAX_FORWARD_POS)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(SHOOTER_WRIST_MAX_REVERSE_POS);

    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(Hardware.SHOOTER_CANCODER_ID)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withRotorToSensorRatio(1)
        .withSensorToMechanismRatio(1);

    public static final MagnetSensorConfigs MAGNET_SENSOR_CONFIGS = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(0);
    
    public static final MotorOutputConfigs BRAKE_CONFIG = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    
    public static final double WRIST_VELOCITY = 4;

    public static final double SHOOTER_MOTOR_ACCELERATION = 40;

    // Motor Enum
    public enum MOTOR
    {
        MOTOR_1,
        MOTOR_2,
        WRIST_MOTOR
    }
}
