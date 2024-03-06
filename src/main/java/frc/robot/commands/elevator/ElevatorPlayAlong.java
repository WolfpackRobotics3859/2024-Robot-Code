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
  
  private double m_PreviousElevatorPosition = 0;

  public ElevatorPlayAlong(Orchestrator orchestrator, Elevator elevator)
  {
    this.m_Orchestrator = orchestrator;
    this.m_Elevator = elevator;
    addRequirements(m_Elevator);
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
    double goalPosition = this.m_Orchestrator.getElevatorPosition();
    if(goalPosition != m_PreviousElevatorPosition)
    {
      m_PreviousElevatorPosition = goalPosition;
      this.m_Elevator.elevatorRequest(ElevatorConstants.MODE.POSITION, goalPosition);
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
