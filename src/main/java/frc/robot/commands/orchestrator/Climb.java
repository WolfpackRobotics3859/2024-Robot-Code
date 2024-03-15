// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Orchestrator;

public class Climb extends Command 
{
  private final Orchestrator m_Orchestrator;
  private final Elevator m_Elevator;
  private final Supplier<Double> m_ElevatorMovementSupplier;

  public Climb(Orchestrator orchestrator, Elevator elevator, Supplier<Double> movementSupplier) 
  {
    this.m_Orchestrator = orchestrator;
    this.m_Elevator = elevator;
    this.m_ElevatorMovementSupplier = movementSupplier;

    addRequirements(m_Orchestrator, m_Elevator);
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
    return false;
  }
}
