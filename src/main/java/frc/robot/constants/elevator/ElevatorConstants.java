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
    public static final double BAR = 0.488;
    public static final double BAR_TOP_CLEAR = 0.675; // The first position where the shooter can freely rotate again above the crossbar.
    public static final double BAR_BOTTOM_CLEAR = 0.372; // The first position where the shooter can freely rotate again below the crossbar.
    public static final double CLOSED_LOOP_ERROR_TOLERANCE = 0.005;

    // Limit Values
    public static final double ELEVATOR_TOP_LIMIT = .84;
    public static final double ELEVATOR_BOTTOM_LIMIT = 0.06;

    public static final double ELEVATOR_BUMPER_SHOT_POSITION = 0.127;
    public static final double ELEVATOR_MANUAL_SHOT_POSITION = 0.127;
    public static final double ELEVATOR_PURGE_POSITION = 0.125;
    public static final double ELEVATOR_CLIMB_SAFE_DOWN = 0.185;
    public static final double ELEVATOR_CLIMB_WRIST_KILL_POSITION = 0.45;

    public static final double ELEVATOR_FEED_FORWARD = 0.2; 

    /** Gains for the elevator motors */
    public static final Slot0Configs ELEVATOR_GAINS = new Slot0Configs()
        .withKP(160).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withKG(0).withGravityType(GravityTypeValue.Elevator_Static);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(1)
        .withMotionMagicAcceleration(0.3);

    /** Software limits for the elevator */
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

    public static final MotorOutputConfigs BRAKE_CONFIG = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(26)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(2)
        .withSupplyCurrentLimitEnable(true);

    public static final MagnetSensorConfigs MAGNET_SENSOR_CONFIGS = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-0.703)
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    
    public static final TalonFXConfiguration ELEVATOR_MOTOR_CONFIG = new TalonFXConfiguration()
        .withSlot0(ELEVATOR_GAINS)
        .withMotionMagic(MOTION_MAGIC_CONFIGS)
        .withSoftwareLimitSwitch(SOFT_LIMIT_CONFIGS)
        .withFeedback(FEEDBACK_CONFIGS)
        .withMotorOutput(BRAKE_CONFIG)
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS);

    public static final CANcoderConfiguration ELEVATOR_CANCODER_CONFIGURATION = new CANcoderConfiguration()
        .withMagnetSensor(MAGNET_SENSOR_CONFIGS);

    public static final double ELEVATOR_POSITION_TOLERANCE = 0.004;

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