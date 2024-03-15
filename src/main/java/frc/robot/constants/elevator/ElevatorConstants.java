package frc.robot.constants.elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
    public static final double BAR = 0.481201171875;
    public static final double BAR_TOP_CLEAR = 0.625; // The first position where the shooter can freely rotate again above the crossbar.
    public static final double BAR_BOTTOM_CLEAR = 0.3330078125; // The first position where the shooter can freely rotate again below the crossbar.
    public static final double CLOSED_LOOP_ERROR_TOLERANCE = 0.005;

    // Limit Values
    public static final double ELEVATOR_TOP_LIMIT = 0.82;
    public static final double ELEVATOR_BOTTOM_LIMIT = 0.06;

    // Misc Values
    public static final double ELEVATOR_FEED_FORWARD = 0.2; 

    // Motor Configs
    public static final Slot0Configs ELEVATOR_GAINS = new Slot0Configs()
        .withKP(160).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withKG(0).withGravityType(GravityTypeValue.Elevator_Static);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(1)
        .withMotionMagicAcceleration(0.3);

    public static final SoftwareLimitSwitchConfigs SOFT_LIMIT_CONFIGS = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ELEVATOR_TOP_LIMIT)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ELEVATOR_BOTTOM_LIMIT);

    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(Hardware.ELEVATOR_CANCODER_ID)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withRotorToSensorRatio(1)
        .withSensorToMechanismRatio(1);

    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    public static final MotorOutputConfigs BRAKE_CONFIG = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(26)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(10)
        .withSupplyCurrentLimitEnable(true);

    // CANCoder Configs
    public static final MagnetSensorConfigs MAGNET_SENSOR_CONFIGS = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-0.14)
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    
    // Condensed Configs
    public static final TalonFXConfiguration ELEVATOR_MOTOR_1_CONFIG = new TalonFXConfiguration()
        .withSlot0(ELEVATOR_GAINS)
        .withMotionMagic(MOTION_MAGIC_CONFIGS)
        .withSoftwareLimitSwitch(SOFT_LIMIT_CONFIGS)
        .withFeedback(FEEDBACK_CONFIGS)
        .withMotorOutput(MOTOR_OUTPUT_CONFIG)
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS);

    public static final TalonFXConfiguration ELEVATOR_MOTOR_2_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withMotorOutput(BRAKE_CONFIG);

    public static final CANcoderConfiguration ELEVATOR_CANCODER_CONFIGURATION = new CANcoderConfiguration()
        .withMagnetSensor(MAGNET_SENSOR_CONFIGS);

    /**
     * @brief The modes available for controlling the elevator.
     */
    public static enum MODE
    {
        PERCENT,
        VOLTAGE,
        POSITION,
        BRAKE
    }
}