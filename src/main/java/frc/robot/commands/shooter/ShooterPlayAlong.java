// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Orchestrator;
import frc.robot.subsystems.Shooter;

public class ShooterPlayAlong extends Command {
  Shooter m_Shooter;
  Orchestrator m_Orchestrator;

  public ShooterPlayAlong(Shooter shooter, Orchestrator orchestrator)
  {
    this.m_Shooter = shooter;
    this.m_Orchestrator = orchestrator;
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // Intentionally Empty
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
    // Intentionally Empty
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
