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

  private double m_PreviousWristPosition;
  private double m_PreviousRollersVelocity;

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
    // prevent null values
    m_PreviousWristPosition = 0;
    m_PreviousRollersVelocity = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if (m_PreviousWristPosition != m_Orchestrator.m_DesiredIntakeWristPosition)
    {
      m_Intake.setWristPosition(m_Orchestrator.m_DesiredIntakeWristPosition);
    }

    if (m_PreviousRollersVelocity != m_Orchestrator.m_DesiredIntakeRollersVelocity)
    {
      m_Intake.setRollersVelocity(m_Orchestrator.m_DesiredIntakeRollersVelocity);
    }

    m_PreviousWristPosition = m_Orchestrator.m_DesiredIntakeWristPosition;
    m_PreviousRollersVelocity = m_Orchestrator.m_DesiredIntakeRollersVelocity;
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
