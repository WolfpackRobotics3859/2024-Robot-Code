// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.shuffletabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Orchestrator;
import frc.robot.utils.Tabs;

/** Add your docs here. */
public class Auto extends Tabs 
{
    private final Orchestrator m_Orchestrator;
    private final Drivetrain m_Drivetrain;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public Auto (Orchestrator orchestrator, Drivetrain drivetrain)
    {
        this.m_Orchestrator = orchestrator;
        this.m_Drivetrain = drivetrain;
    }

    @Override
    public void createEntries()
    {
        tab = Shuffleboard.getTab("Auto");
        
        autoChooser.setDefaultOption("None", new PrintCommand("No auto selected."));

        tab.add("Auto Chooser", autoChooser);
    }

    @Override
    public void update()
    {
        // Intentionally Empty        
    }

    public SendableChooser<Command> getChooser()
    {
        return this.autoChooser;
    }
}
