// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class KillIntake extends Command
{
  private final Intake m_Intake;

  public KillIntake(Intake intake)
  {
    this.m_Intake = intake;

    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Intake.setRollerVoltage(0);
    m_Intake.setWristPercent(0);
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
