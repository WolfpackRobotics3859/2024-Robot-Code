// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ZeroIntake extends Command 
{
  private final Intake m_Intake;
  private final Timer m_Timer;

  /** Creates a new ZeroIntake. */
  public ZeroIntake(Intake intake)
  {
    this.m_Intake = intake;
    
    addRequirements(m_Intake);
    
    this.m_Timer = new Timer();
    m_Timer.start();
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
    m_Intake.setWristPercent(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_Intake.setWristZero();
    m_Intake.setWristPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
