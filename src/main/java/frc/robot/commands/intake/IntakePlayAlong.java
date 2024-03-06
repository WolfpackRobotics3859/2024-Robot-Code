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

  private double m_PreviousWristPosition = 0;
  private double m_PreviousRollersVoltage = 0;

  public IntakePlayAlong(Orchestrator orchestrator, Intake intake)
  {
    this.m_Orchestrator = orchestrator;
    this.m_Intake = intake;

    addRequirements(m_Intake);
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
    double wristPosition = m_Orchestrator.getIntakePosition();
    double rollerVoltage = m_Orchestrator.getIntakeRollersVoltage();
    if(wristPosition != m_PreviousWristPosition)
    {
      m_Intake.setWristPosition(wristPosition);
      m_PreviousWristPosition = wristPosition;
    }
    if(rollerVoltage != m_PreviousRollersVoltage)
    {
      m_Intake.setRollerVoltage(rollerVoltage);
      m_PreviousRollersVoltage = rollerVoltage;
    }
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
