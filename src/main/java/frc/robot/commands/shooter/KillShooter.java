// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.constants.shooter.ShooterConstants;

public class KillShooter extends Command
{
  private final Shooter m_Shooter;

  public KillShooter(Shooter shooter)
  {
    this.m_Shooter = shooter;
    addRequirements(m_Shooter);
  }

  @Override
  public void initialize()
  {
    m_Shooter.setMotor(ShooterConstants.MOTOR.WRIST_MOTOR, ShooterConstants.MODE.PERCENT, 0);
    m_Shooter.setMotor(ShooterConstants.MOTOR.MOTOR_1, ShooterConstants.MODE.PERCENT, 0);
    m_Shooter.setMotor(ShooterConstants.MOTOR.MOTOR_2, ShooterConstants.MODE.PERCENT, 0);
    m_Shooter.setMotor(ShooterConstants.MOTOR.FEEDER_MOTOR, ShooterConstants.MODE.PERCENT, 0);
  }

  @Override
  public void execute()
  { 
    // Intentionally Empty
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
