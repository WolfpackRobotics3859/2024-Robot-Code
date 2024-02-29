// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Orchestrator;

public class ZeroIntake extends Command 
{
  private final Orchestrator m_Orchestrator;
  private final Intake m_Intake;
  private final Timer m_Timer;

  /** Creates a new ZeroIntake. */
  public ZeroIntake(Orchestrator orchestrator, Intake intake)
  {
    this.m_Orchestrator = orchestrator;
    this.m_Intake = intake;
    
    addRequirements(m_Orchestrator, m_Intake);
    
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
    m_Intake.setWristPercent(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_Intake.zeroWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    if (m_Timer.get() > 1.5)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
