// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class Orchestrator
{
    private final Drivetrain m_Drivetrain;
    private final Shooter m_Shooter;
    private final Elevator m_Elevator;
    private final IntakeSubsystem m_Intake;

    /**
     * Creates a new orchestrator class
     * @param drivetrain The drivetrain subsystem
     * @param shooter The shooter subsystem
     * @param elevator The elevator subsytem
     * @param intake The intake subsystem
     */
    public Orchestrator(Drivetrain drivetrain, Shooter shooter, Elevator elevator, IntakeSubsystem intake)
    {
        this.m_Drivetrain = drivetrain;
        this.m_Shooter = shooter;
        this.m_Elevator = elevator;
        this.m_Intake = intake;
    }
}
