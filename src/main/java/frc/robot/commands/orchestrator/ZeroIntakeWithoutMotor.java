// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class ZeroIntakeWithoutMotor extends InstantCommand
{
  private final Intake m_Intake;

  public ZeroIntakeWithoutMotor(Intake intake) 
  {
    this.m_Intake = intake;

    ignoringDisable(true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Intake.zeroWrist();
  }

  @Override
  public boolean runsWhenDisabled()
  {
    return true;
  }
}
