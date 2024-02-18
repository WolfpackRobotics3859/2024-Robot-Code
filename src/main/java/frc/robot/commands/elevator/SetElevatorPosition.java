// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.*;

public class SetElevatorPosition extends Command
{
  Elevator m_Elevator;
  double m_Position;

  /**
   * @brief Sends the elevator motors toward a given position.
   * @param elevator The elevator subsystem object.
   * @param position The position to send the elevator toward.
  */
  public SetElevatorPosition(Elevator elevator, double position)
  {
    this.m_Elevator = elevator;
    this.m_Position = position;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Elevator.setElevatorPosition(this.m_Position);
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
    // if command was interrupted set elevator to stop
    if (interrupted)
    {
      m_Elevator.setElevatorPercent(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    // if elevator is at set position end command
    if (Util.epsilonEquals(m_Elevator.getElevatorPosition().getValueAsDouble(), m_Position, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
