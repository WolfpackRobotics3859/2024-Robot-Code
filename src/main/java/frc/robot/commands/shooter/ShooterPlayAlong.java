// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.shooter.ShooterConstants.MOTOR;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Shooter;

public class ShooterPlayAlong extends Command
{
  private final Shooter m_Shooter;
  private final Orchestrator m_Orchestrator;

  private double m_PreviousWristPosition;
  private double m_PreviousShooter1Velocity;
  private double m_PreviousShooter2Velocity;
  private double m_PreviousFeederVoltage;

  /** Creates a new ShooterPlayAlong. */
  public ShooterPlayAlong(Orchestrator orchestrator, Shooter shooter)
  {
    this.m_Orchestrator = orchestrator;
    this.m_Shooter = shooter;

    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // prevent null values
    m_PreviousWristPosition = 0;
    m_PreviousShooter1Velocity = 0;
    m_PreviousShooter2Velocity = 0;
    m_PreviousFeederVoltage = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if (m_PreviousWristPosition != m_Orchestrator.m_DesiredShooterWristPosition)
    {
      m_Shooter.setWristPosition(m_Orchestrator.m_DesiredShooterWristPosition);
    }

    if (m_PreviousShooter1Velocity != m_Orchestrator.m_DesiredShooterMotor1Velocity)
    {
      m_Shooter.setMotorVelocity(MOTOR.MOTOR_1, m_Orchestrator.m_DesiredShooterMotor1Velocity);
    }

    if (m_PreviousShooter2Velocity != m_Orchestrator.m_DesiredShooterMotor2Velocity)
    {
      m_Shooter.setMotorVelocity(MOTOR.MOTOR_2, m_Orchestrator.m_DesiredShooterMotor2Velocity);
    }

    if (m_PreviousFeederVoltage != m_Orchestrator.m_DesiredShooterFeederVoltage)
    {
      m_Shooter.setFeederVoltage(m_Orchestrator.m_DesiredShooterFeederVoltage);
    }

    m_PreviousWristPosition = m_Orchestrator.m_DesiredShooterWristPosition;
    m_PreviousShooter1Velocity = m_Orchestrator.m_DesiredShooterMotor1Velocity;
    m_PreviousShooter2Velocity = m_Orchestrator.m_DesiredShooterMotor2Velocity;
    m_PreviousFeederVoltage = m_Orchestrator.m_DesiredShooterFeederVoltage;
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
