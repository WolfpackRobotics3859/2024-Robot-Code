// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Shooter;

public class AmpShotAuto extends Command 
{
  private final Orchestrator m_Orchestrator;
  private final Shooter m_Shooter;
  private final Timer m_Timer = new Timer();
  /** Creates a new AmpShot. */
  public AmpShotAuto(Orchestrator orchestrator, Shooter shooter) 
  {
    this.m_Orchestrator = orchestrator;
    this.m_Shooter = shooter;

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
    m_Orchestrator.shootAmp();

    if (m_Shooter.shooterClear())
    {
      m_Timer.start();
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
  public boolean isFinished()
  {
    if (m_Timer.get() > 1)
    {
      return true;
    }
    return false;
  }
}
