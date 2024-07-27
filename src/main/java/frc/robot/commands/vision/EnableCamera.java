// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.Vision;

public class EnableCamera extends InstantCommand
{
  private final Vision m_Vision;
  private final int m_Index;

  /**
   * Enables a vision camera.
   * @param vision The vision system
   * @param index The index of the camera to enable
   */
  public EnableCamera(Vision vision, int index)
  {
    this.m_Vision = vision;
    this.m_Index = index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Vision.setCamEnabled(m_Index, true);
  }
}
