package frc.robot.constants.shooter;

import com.ctre.phoenix6.configs.Slot1Configs;


public class ShooterConstants
{
    public static final int SHOOTER_MOTOR_1_ID = 9;
    public static final int SHOOTER_MOTOR_2_ID = 10;

    // Gains for the shooter motors
    public static final Slot1Configs SHOOTER_GAINS = new Slot1Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

}
