// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

/** Add your docs here. */
public class IntakeConstants
{
    public static final Slot0Configs INTAKE_GAINS = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0);

    public static final Slot0Configs INTAKE_WRIST_GAINS = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(10)
        .withMotionMagicAcceleration(2);

    public static final double INTAKE_WRIST_FEED_FORWARD = 0.05;
}

   
