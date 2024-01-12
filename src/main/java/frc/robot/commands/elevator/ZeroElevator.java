// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command {
  Elevator m_elevator;
  
  /**
   * @brief Moves the elevator down at a fixed rate until it hits the limit switch at the bottom, where it sets it's position to 0.
   * @param elevator The elevator subsystem object.
   */
  public ZeroElevator(Elevator elevator)
  {
    this.m_elevator = elevator;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    // Send the elevator down
    m_elevator.setElevatorPercent(-.1);
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
    // Set the elevator to neutral
    m_elevator.setElevatorPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Get the limit switch value and end the command if the switch is closed
    if (m_elevator.getLimitSwitch() == ReverseLimitValue.ClosedToGround)
    {
      return true;
    }
    return false;
  }
}
