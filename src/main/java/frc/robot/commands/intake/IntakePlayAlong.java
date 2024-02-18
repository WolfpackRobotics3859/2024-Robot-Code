// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Intake;

public class IntakePlayAlong extends Command {
  Intake m_Intake;
  Orchestrator m_Orchestrator;

  public IntakePlayAlong(Intake intake, Orchestrator orchestrator)
  {
    this.m_Intake = intake;
    this.m_Orchestrator = orchestrator;
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
    m_Orchestrator.updateIntakeOdometry
    (
      m_Intake.getWristPosition().getValueAsDouble()
    );
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
