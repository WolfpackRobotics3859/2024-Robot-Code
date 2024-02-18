// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Util;

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Shooter.setWristPosition(this.m_Position);
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
    // if command was interrupted stop the shooter (set the desired position to just wherever it is at the moment)
    if (interrupted)
    {
      m_Shooter.setWristPosition(m_Shooter.getWristMotorPosition().getValueAsDouble());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    // if shooter wrist is at set position end the command
    if (Util.epsilonEquals(m_Shooter.getWristMotorPosition().getValueAsDouble(), m_Position, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
