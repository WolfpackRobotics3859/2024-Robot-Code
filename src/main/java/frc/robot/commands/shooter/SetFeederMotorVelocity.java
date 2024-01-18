// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.shooter.ShooterConstants.MOTOR;
import frc.robot.subsystems.Shooter;

public class SetFeederMotorVelocity extends Command
{
  Shooter m_Shooter;
  double m_Velocity;

  /**
   * @brief Runs the feeder motor at a given velocity.
   * @param shooter The shooter subsystem object.
   * @param velocity The desired velocity to run the motor at, measured in rotations per second.
  */
  public SetFeederMotorVelocity(Shooter shooter, double velocity)
  {
    this.m_Shooter = shooter;
    this.m_Velocity = velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
     m_Shooter.setMotorVelocity(MOTOR.FEEDER_MOTOR, this.m_Velocity);
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
    m_Shooter.setShooterMotorPercent(MOTOR.FEEDER_MOTOR, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
