package frc.robot.constants.elevator;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

public class ElevatorConstants
{
    /** Gains for the elevator motors */
    public static final Slot0Configs ELEVATOR_GAINS = new Slot0Configs()
        .withKP(0).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    /** The velocity at which the elevator moves between positions, measured in rotations per second.  */
    public static final double ELEVATOR_VELOCITY = 4;

    /** The maximum allowed position the elevator can extend forward to */
    public static final double ELEVATOR_MAX_FORWARD_POS = 60;

    /** Software limits for the elevator */
    public static final SoftwareLimitSwitchConfigs SOFT_LIMIT_CONFIGS = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ELEVATOR_MAX_FORWARD_POS);

    /** Limit switch configuration */
    public static final HardwareLimitSwitchConfigs HARD_LIMIT_CONFIGS = new HardwareLimitSwitchConfigs()
        .withReverseLimitAutosetPositionEnable(true)
        .withReverseLimitAutosetPositionValue(0)
        .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
        .withReverseLimitEnable(true)
        .withReverseLimitRemoteSensorID(Hardware.ELEVATOR_LIMIT_SWITCH_ID);
}
