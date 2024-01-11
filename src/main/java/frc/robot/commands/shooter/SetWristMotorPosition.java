// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetWristMotorPosition extends Command
{
  Shooter m_Shooter;
  double m_Position;

  /**
   * @brief Spins the wrist motor towards a given position.
   * @param shooter The shooter subsystem object.
   * @param velocity The desired position to send the motor towards, measured in rotations.
  */
  public SetWristMotorPosition(Shooter shooter, double position)
  {
    this.m_Shooter = shooter;
    this.m_Position = position;
    addRequirements(m_Shooter);
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
    m_Shooter.setWristPos(m_Position);
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
    return false;
  }
}
