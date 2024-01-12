package frc.robot.constants.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ShooterConstants
{
    // Gains for shooter motor 1
    public static final Slot0Configs SHOOTER_1_GAINS = new Slot0Configs()
        .withKP(0.015365).withKI(0).withKD(0)
        .withKS(0.035335).withKV(0.11133).withKA(0.0098933);

    // Gains for shooter motor 2
    public static final Slot0Configs SHOOTER_2_GAINS = new Slot0Configs()
        .withKP(0.015365).withKI(0).withKD(0)
        .withKS(0.035335).withKV(0.11133).withKA(0.0098933);
    
    // Gains for the wrist motor
    public static final Slot0Configs WRIST_GAINS = new Slot0Configs()
        .withKP(0.03).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // Motor Enum
    public enum MOTOR
    {
        MOTOR_1,
        MOTOR_2,
        WRIST_MOTOR
    }
}
