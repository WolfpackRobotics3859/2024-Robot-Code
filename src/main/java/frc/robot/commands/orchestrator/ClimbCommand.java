// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.elevator.ElevatorConstants.MODE;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;

public class ClimbCommand extends Command 
{
  // private final Climb m_Climb;
  // private final Supplier<Double> m_MovementSupplier;

  // public ClimbCommand(Climb climb, Supplier<Double> movementSupplier) 
  // {
  //   this.m_Climb = climb;
  //   this.m_MovementSupplier = movementSupplier;

  //   addRequirements(m_Climb);
  // }

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
