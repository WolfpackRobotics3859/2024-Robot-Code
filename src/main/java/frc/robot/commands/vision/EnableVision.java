// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.Vision;

public class EnableVision extends InstantCommand
{
  private final Vision m_Vision;

  public EnableVision(Vision vision)
  {
    this.m_Vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Vision.setVisionEnabled(true);
  }
}
