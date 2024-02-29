// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Orchestrator;

public class ElevatorPlayAlong extends Command
{
  private final Orchestrator m_Orchestrator;
  private final Elevator m_Elevator;
  private final Supplier<Double> m_ElevatorPercent;
  
  private double m_PreviousElevatorPosition;

  /** Creates a new ElevatorPlayAlong. */
  public ElevatorPlayAlong(Orchestrator orchestrator, Elevator elevator, Supplier<Double> elevatorPercent)
  {
    this.m_Orchestrator = orchestrator;
    this.m_Elevator = elevator;
    this.m_ElevatorPercent = elevatorPercent;

    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // prevent null numbers
    this.m_PreviousElevatorPosition = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if (!m_Orchestrator.climbing)
    {
      // if the last position the elevator was told to go to is not the same as the desired position
      if (this.m_PreviousElevatorPosition != m_Orchestrator.m_DesiredElevatorPosition)
      {
        m_Elevator.setElevatorPosition(m_Orchestrator.m_DesiredElevatorPosition);
      }

      this.m_PreviousElevatorPosition = m_Orchestrator.m_DesiredElevatorPosition;
    }
    else
    {
      if (m_Elevator.getElevatorPosition().getValueAsDouble() > ElevatorConstants.ELEVATOR_CLIMB_SAFE_DOWN || m_ElevatorPercent.get() >= 0)
      {
        m_Elevator.setElevatorPercent(m_ElevatorPercent.get() * 0.5);
      }
      else
      {
        m_Elevator.setElevatorBrake();
      }
    }
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
