// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.shooter.ShooterConstants.MOTOR;
import frc.robot.subsystems.Shooter;

public class SetShooterMotorsVelocity extends Command
{
  Shooter m_Shooter;
  double m_Velocity1;
  double m_Velocity2;

  /**
   * @brief Runs motor number 1 at a given velocity.
   * @param shooter The shooter subsystem object.
   * @param velocity The desired velocity to run the motor at, measured in rotations per second.
  */
  public SetShooterMotorsVelocity(Shooter shooter, double velocity1, double velocity2)
  {
    this.m_Shooter = shooter;
    this.m_Velocity1 = velocity1;
    this.m_Velocity2 = velocity2;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
     m_Shooter.setMotorVelocity(MOTOR.MOTOR_1, this.m_Velocity1);
     m_Shooter.setMotorVelocity(MOTOR.MOTOR_2, this.m_Velocity2);
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
    m_Shooter.setShooterMotorPercent(MOTOR.MOTOR_1, 0);
    m_Shooter.setShooterMotorPercent(MOTOR.MOTOR_2, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
