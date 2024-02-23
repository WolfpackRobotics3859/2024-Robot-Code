// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Shooter;

public class ShooterPlayAlong extends Command
{
  private final Shooter m_Shooter;
  private final Orchestrator m_Orchestrator;

  /** Creates a new ShooterPlayAlong. */
  public ShooterPlayAlong(Orchestrator orchestrator, Shooter shooter)
  {
    this.m_Orchestrator = orchestrator;
    this.m_Shooter = shooter;
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
    // feed shooter motor velocities and wrist position here
    // make sure to add logic to prevent redundant motor requests
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    // Intentionally Empty
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
