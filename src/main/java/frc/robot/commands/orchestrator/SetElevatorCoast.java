// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class SetElevatorCoast extends InstantCommand
{
  private final Elevator m_Elevator;

  public SetElevatorCoast(Elevator elevator)
  {
    this.m_Elevator = elevator;

    addRequirements(m_Elevator);
    ignoringDisable(true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Elevator.setBrakeMode(false);
  }
}
