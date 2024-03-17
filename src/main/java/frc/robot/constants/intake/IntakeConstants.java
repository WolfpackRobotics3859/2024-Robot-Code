// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

/** Add your docs here. */
public class IntakeConstants
{
    // Misc Values
    public static final double INTAKE_WRIST_FEED_FORWARD = 0.2;
    public static final double INTAKE_ROLLERS_ACCELERATION = 30;

    // Motor Configs
    public static final Slot0Configs INTAKE_ROLLER_GAINS = new Slot0Configs()
        .withKP(0.089884).withKI(0).withKD(0)
        .withKS(0.49503).withKV(0.11396).withKA(0.0043837);

    public static final Slot0Configs INTAKE_WRIST_GAINS = new Slot0Configs()
        .withKP(20).withKI(0).withKD(0)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final MotionMagicConfigs WRIST_MOTOR_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(30)
        .withMotionMagicAcceleration(25);

    // Condensed Configs
    public static final TalonFXConfiguration INTAKE_WRIST_CONFIGURATION = new TalonFXConfiguration()
        .withMotionMagic(WRIST_MOTOR_MOTION_MAGIC_CONFIGS)
        .withSlot0(INTAKE_WRIST_GAINS);

    public static final TalonFXConfiguration INTAKE_ROLLER_CONFIGURATION = new TalonFXConfiguration()
        .withSlot0(INTAKE_ROLLER_GAINS);
}

   
