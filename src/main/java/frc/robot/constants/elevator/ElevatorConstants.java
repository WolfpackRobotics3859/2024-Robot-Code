package frc.robot.constants.elevator;

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

public class ElevatorConstants
{
    /** Gains for the elevator motors */
    public static final Slot0Configs ELEVATOR_GAINS = new Slot0Configs()
        .withKP(1).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(10)
        .withMotionMagicAcceleration(2);

    /** The velocity at which the elevator moves between positions, measured in rotations per second.  */
    public static final double ELEVATOR_FEED_FORWARD = 0.03;

    /** The maximum allowed position the elevator can extend forward to */
    public static final double ELEVATOR_MAX_FORWARD_POS = .2;
    public static final double ELEVATAOR_MAX_REVERSE_POS = 0;

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

    public static final MagnetSensorConfigs MAGNET_SENSOR_CONFIGS = new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(0.269);
    
    public static final MotorOutputConfigs BRAKE_CONFIG = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
}
