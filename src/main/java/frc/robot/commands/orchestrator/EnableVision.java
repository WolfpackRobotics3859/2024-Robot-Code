// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class EnableVision extends InstantCommand
{
  private final Drivetrain m_Drivetrain;

  public EnableVision(Drivetrain drivetrain)
  {
    this.m_Drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Drivetrain.setVisionEnabled(true);
  }
}
