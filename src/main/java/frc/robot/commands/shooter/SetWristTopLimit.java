// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.constants.shooter.ShooterConstants.MOTOR;

public class SetWristTopLimit extends Command
{
  Shooter m_Shooter;
  double m_Limit;

  /**
   * @brief Applies a percent of the available voltage to the wrist motor.
   * @param shooter The shooter subsystem object.
   * @param percent The percent voltage to apply to the motor, from -1 to 1.
  */
  public SetWristTopLimit(Shooter shooter, double limit)
  {
    this.m_Shooter = shooter;
    this.m_Limit = limit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Shooter.setShooterMotorPercent(MOTOR.WRIST_MOTOR, this.m_Limit);
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
    m_Shooter.setShooterMotorPercent(MOTOR.WRIST_MOTOR, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
