// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.elevator.ElevatorConstants.MODE;
import frc.robot.subsystems.Elevator;

public class Climb extends Command 
{
  private final Elevator m_Elevator;
  private final Supplier<Double> m_ElevatorMovementSupplier;

  public Climb(Elevator elevator, Supplier<Double> movementSupplier) 
  {
    this.m_Elevator = elevator;
    this.m_ElevatorMovementSupplier = movementSupplier;

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
    if (m_ElevatorMovementSupplier.get() != 0)
    {
      m_Elevator.elevatorRequest(MODE.PERCENT, m_ElevatorMovementSupplier.get() * 0.7);
    }
    else
    {
      m_Elevator.elevatorRequest(MODE.BRAKE, 0);
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
