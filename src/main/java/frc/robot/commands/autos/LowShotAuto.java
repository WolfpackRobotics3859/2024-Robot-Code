// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Shooter;

public class LowShotAuto extends Command {
  private final Orchestrator m_Orchestrator;
  private final Shooter m_Shooter;
  private Timer m_Timer = new Timer();

  public LowShotAuto(Orchestrator orchestrator, Shooter shooter) 
  {
    m_Orchestrator = orchestrator;
    m_Shooter = shooter;

    addRequirements(m_Orchestrator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Orchestrator.freshenOrchestrator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_Orchestrator.shootLow();

    if(m_Shooter.shooterClear())
    {
      m_Timer.start();
    }
    else
    {
      m_Timer.stop();
      m_Timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_Timer.stop();
    m_Timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return m_Timer.hasElapsed(0.5);
  }
}