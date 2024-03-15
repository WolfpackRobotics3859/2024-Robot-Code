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

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.constants.Hardware;

public class ShooterConstants
{
    // Position Values
    public static final double WRIST_CLEARANCE_POSITION = .68;

    // Limit Values
    public static final double WRIST_MAX_DOWN_POSITION = 0.3;
    public static final double WRIST_MAX_UP_POSIION = 0.71;
    
    // Accelerations
    public static final double SHOOTER_MOTOR_ACCELERATION = 30;

    // Misc values
    public static final double SHOOTER_WRIST_FEED_FORWARD = 0.3;

    // Motor Configs
    public static final Slot0Configs SHOOTER_1_GAINS = new Slot0Configs()
        .withKP(0.094562).withKI(0).withKD(0)
        .withKS(0.095166).withKV(0.11334).withKA(0.023285);

    public static final Slot0Configs SHOOTER_2_GAINS = new Slot0Configs()
        .withKP(0.090537).withKI(0).withKD(0)
        .withKS(0.090537).withKV(0.11444).withKA(0.021771);
    
    public static final Slot0Configs WRIST_GAINS = new Slot0Configs()
        .withKP(37).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final Slot0Configs FEEDER_GAINS = new Slot0Configs()
        .withKP(0.5).withKI(0).withKD(0)
        .withKS(0.15837).withKV(0.10739).withKA(0.0018114);

    public static final MotionMagicConfigs WRIST_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(3)
        .withMotionMagicAcceleration(1);

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
    
    // CANCoder configs
    public static final MagnetSensorConfigs WRIST_CANCODER_MAGNET_CONFIGS = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(0.26)
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);

    // Condensed Configs
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

    public enum MODE
    {
        VELOCITY,
        PERCENT,
        VOLTAGE,
        BRAKE,
        POSITION
    }

    // Tolerances
    public static final double VELOCITY_CLOSED_LOOP_ERROR_TOLERANCE = 1;
    public static final double POSITION_CLOSED_LOOP_ERROR_TOLERANCE = 0.04;

    // Shooting positions
    // TODO fix these values
    public static final Measure<Distance> MIN_SHOOTING_DISTANCE = Units.Inches.of(0);
    public static final Measure<Distance> MAX_SHOOTING_DISTANCE = Units.Inches.of(120);
}