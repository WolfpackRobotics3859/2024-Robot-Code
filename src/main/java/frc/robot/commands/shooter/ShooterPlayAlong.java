// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Shooter;
import frc.robot.constants.shooter.ShooterConstants;

public class ShooterPlayAlong extends Command
{
  private final Shooter m_Shooter;
  private final Orchestrator m_Orchestrator;

  private double m_PreviousWristPosition = 0;
  private double m_PreviousShooter1Velocity = 0;
  private double m_PreviousShooter2Velocity = 0;
  private double m_PreviousFeederVoltage = 0;

  /** Creates a new ShooterPlayAlong. */
  public ShooterPlayAlong(Orchestrator orchestrator, Shooter shooter)
  {
    this.m_Orchestrator = orchestrator;
    this.m_Shooter = shooter;

    addRequirements(m_Shooter);
  }

  @Override
  public void initialize()
  {
    // Intentionally Empty
  }

  @Override
  public void execute()
  { 
    double wristPosition = m_Orchestrator.getShooterAngle();
    double shooter1Velocity = m_Orchestrator.getShooterTopRollerVelocity();
    double shooter2Velocity = m_Orchestrator.getShooterBottomRollerVelocity();
    double feederVoltage = m_Orchestrator.getShooterFeederVoltage();

    if(wristPosition != m_PreviousWristPosition)
    {
      m_Shooter.setMotor(ShooterConstants.MOTOR.WRIST_MOTOR, ShooterConstants.MODE.POSITION, wristPosition);
      m_PreviousWristPosition = wristPosition;
    }
    if(shooter1Velocity != m_PreviousShooter1Velocity)
    {
      m_Shooter.setMotor(ShooterConstants.MOTOR.MOTOR_1, ShooterConstants.MODE.VELOCITY, shooter1Velocity);
      m_PreviousShooter1Velocity = shooter1Velocity;
    }
    if(shooter2Velocity != m_PreviousShooter2Velocity)
    {
      m_Shooter.setMotor(ShooterConstants.MOTOR.MOTOR_2, ShooterConstants.MODE.VELOCITY, shooter2Velocity);
      m_PreviousShooter2Velocity = shooter2Velocity;
    }
    if(feederVoltage != m_PreviousFeederVoltage)
    {
      m_Shooter.setMotor(ShooterConstants.MOTOR.FEEDER_MOTOR, ShooterConstants.MODE.VOLTAGE, feederVoltage);
      m_PreviousFeederVoltage = feederVoltage;
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    // Intentionally Empty
  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
