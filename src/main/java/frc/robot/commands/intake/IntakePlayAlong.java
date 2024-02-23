// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Orchestrator;

public class IntakePlayAlong extends Command
{
  private final Orchestrator m_Orchestrator;
  private final Intake m_Intake;

  /** Creates a new IntakePlayAlong. */
  public IntakePlayAlong(Orchestrator orchestrator, Intake intake)
  {
    this.m_Orchestrator = orchestrator;
    this.m_Intake = intake;
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
    // feed intake wrist positions and roller velocities here
    // must make sure to put some basic logic here to prevent redundant motor requests
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
