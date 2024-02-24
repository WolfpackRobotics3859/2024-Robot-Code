// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Util;

public class SetIntakeRollersVelocity extends Command 
{
  
  private final Intake m_Intake;
  private final double m_Velocity;

  /**
   * @brief Spins the rollers at a given velocity
   * @param shooter The shooter subsystem.
   * @param velocity The desired velocity to run the intake at.
  */
  public SetIntakeRollersVelocity(Intake intake, double velocity)
  {
    this.m_Intake = intake;
    this.m_Velocity = velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Intake.setRollersVelocity(m_Velocity);
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
    // Intentionally Empty
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if (Util.epsilonEquals(m_Intake.getRollerVelocity().getValueAsDouble(), m_Velocity, 2))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
