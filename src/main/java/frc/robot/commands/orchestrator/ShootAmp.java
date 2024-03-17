// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Orchestrator;

public class ShootAmp extends Command 
{
  private final Orchestrator m_Orchestrator;

  public ShootAmp(Orchestrator orchestrator) 
  {
    this.m_Orchestrator = orchestrator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Orchestrator.setAmpShoot(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Intentionally Empty
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_Orchestrator.setAmpShoot(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
