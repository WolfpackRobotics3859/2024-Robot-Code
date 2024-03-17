// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Shooter;

public class IntakeAuto extends Command 
{
  private final Orchestrator m_Orchestrator;
  private final Shooter m_Shooter;

  public IntakeAuto(Orchestrator orchestrator, Shooter shooter) {
    m_Orchestrator = orchestrator;
    m_Shooter = shooter;
    addRequirements(m_Orchestrator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.print("Intake Auto Started");

    // Intentionally Empty
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_Orchestrator.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    System.out.print("Intake Auto Started");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return m_Shooter.hasNoteCentered() || m_Shooter.hasNoteRearPosition();
  }
}
