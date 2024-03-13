// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Orchestrator;

public class ShootLow extends Command 
{
  private final Orchestrator m_Orchestrator;
  private final Drivetrain m_Drivetrain;

  /** Creates a new ShootLow. */
  public ShootLow(Orchestrator orchestrator, Drivetrain drivetrain) 
  {
    this.m_Orchestrator = orchestrator;
    this.m_Drivetrain = drivetrain;
    addRequirements(m_Orchestrator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Orchestrator.freshenOrchestrator();
    m_Drivetrain.setAlignToSpeaker(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_Orchestrator.shootLow();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_Drivetrain.setAlignToSpeaker(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
