package frc.robot.constants.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ShooterConstants
{
    // Gains for the shooter motors
    public static final Slot0Configs SHOOTER_GAINS = new Slot0Configs()
        .withKP(0.35).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    
    // Gains for the wrist motor
    public static final Slot0Configs WRIST_GAINS = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
}
