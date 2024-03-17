// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Orchestrator;

public class ManualControl extends Command 
{
  private Orchestrator m_Orchestrator;
  public ManualControl(Orchestrator orchestrator) 
  {
    m_Orchestrator = orchestrator;
    addRequirements(m_Orchestrator);
  }

  @Override
  public void initialize() 
  {
    // Intentionally Empty
  }

  @Override
  public void execute() 
  {
    m_Orchestrator.manualControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // Intentionally Empty
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
