// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorPercent extends Command
{
  Elevator m_Elevator;
  double m_Percent;

  /**
   * @brief Sets the elevator motors to a given percentage of the available voltage.
   * @param elevator The elevator subsystem object.
   * @param percent The percent voltage to apply to the motors (-1 to 1).
  */
  public SetElevatorPercent(Elevator elevator, double percent)
  {
    this.m_Elevator = elevator;
    this.m_Percent = percent;
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Elevator.setElevatorPercent(this.m_Percent);
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
    m_Elevator.setElevatorPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
