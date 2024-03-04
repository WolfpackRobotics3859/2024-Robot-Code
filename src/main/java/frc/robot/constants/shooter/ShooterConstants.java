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
    public static final double WRIST_CLEARANCE_POSITION = .68;
    public static final double WRIST_DEFAULT_POSITION = 0.49;
    public static final double WRIST_AMP_SHOOTING_POSITION = 0.662598; 
    public static final double WRIST_BUMPER_SHOT_POSITION = 0.6525
    ;
    public static final double WRIST_AUTO_BUMPER_SHOT_POSITION = 0.635;
    public static final double WRIST_MANUAL_SHOT_POSITION = 0.6;
    public static final double WRIST_PURGE_POSITION = 0.58;
    public static final double WRIST_AMP_BACK_SHOT_POSITION = 0.52;


    // Software Limit Values
    public static final double WRIST_MAX_DOWN_POSITION = 0.3;
    public static final double WRIST_MAX_UP_POSIION = 1;

    // Velocities
    public static final double BUMPER_SHOT_VELOCITY = 35;
    public static final double INTAKE_SHOOTERS_VELOCITY = -13;
    public static final double PURGE_VELOCITY = 20;

    // Voltages
    public static final double INTAKE_FEEDER_VOLTAGE = -1.5;
    public static final double FEEDER_SHOOTING_VOLTAGE = 7;
    public static final double FEEDER_AMP_SHOT_VOLTAGE = 3;


    // Wrist values
    public static final double SHOOTER_WRIST_FEED_FORWARD = 0.75;

    // Shooter motor configs
    public static final double SHOOTER_MOTOR_ACCELERATION = 30;

    public static final Slot0Configs SHOOTER_1_GAINS = new Slot0Configs()
        .withKP(0.0025769).withKI(0).withKD(0)
        .withKS(0.17472).withKV(0.11134).withKA(0.029604);

    public static final Slot0Configs SHOOTER_2_GAINS = new Slot0Configs()
        .withKP(0.011753).withKI(0).withKD(0)
        .withKS(0.13771).withKV(0.10834).withKA(0.01631);
    
    // Wrist motor configs
    public static final Slot0Configs WRIST_GAINS = new Slot0Configs()
        .withKP(35).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    // Feeder motor
    public static final Slot0Configs FEEDER_GAINS = new Slot0Configs()
        .withKP(0.5).withKI(0).withKD(0)
        .withKS(0.15837).withKV(0.10739).withKA(0.0018114);

    // Wrist configs
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
    
    // Wrist CANCoder
    public static final MagnetSensorConfigs WRIST_CANCODER_MAGNET_CONFIGS = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(-0.25)
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
    public static final double WRIST_MOVEMENT_TOLERANCE = 0.04;
    public static final double SHOOTER_VELOCITY_TOLERANCE = 3;
}