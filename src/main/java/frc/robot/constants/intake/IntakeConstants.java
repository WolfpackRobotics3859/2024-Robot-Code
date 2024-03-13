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
    // Position Values
    public static final double INTAKE_DOWN_POSITION = 0;
    public static final double INTAKE_UP_POSITION = 4.3;
    public static final double INTAKE_CLEAR_POSITION = 3.0;
    public static final double INTAKE_BUMPER_SHOT_POSITION = 2.7;
    public static final double INTAKE_AMP_SHOT_POSITION = 3.6;

    // Velocities
    public static final double INTAKE_ROLLERS_VELOCITY = -55;
    public static final double PURGE_VELOCITY = 45;

    public static final Slot0Configs INTAKE_ROLLER_GAINS = new Slot0Configs()
        .withKP(0.089884).withKI(0).withKD(0)
        .withKS(0.49503).withKV(0.11396).withKA(0.0043837);

    public static final Slot0Configs INTAKE_WRIST_GAINS = new Slot0Configs()
        .withKP(20).withKI(0).withKD(0)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final MotionMagicConfigs WRIST_MOTOR_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(30)
        .withMotionMagicAcceleration(25);

    public static final TalonFXConfiguration INTAKE_WRIST_CONFIGURATION = new TalonFXConfiguration()
        .withMotionMagic(WRIST_MOTOR_MOTION_MAGIC_CONFIGS)
        .withSlot0(INTAKE_WRIST_GAINS);

    public static final TalonFXConfiguration INTAKE_ROLLER_CONFIGURATION = new TalonFXConfiguration()
        .withSlot0(INTAKE_ROLLER_GAINS);

    public static final double INTAKE_WRIST_FEED_FORWARD = 0.2;
    public static final double INTAKE_ROLLERS_ACCELERATION = 30;

    // Misc
    public static final double INTAKE_WRIST_POSITION_TOLERANCE = 0.2;
}

   
