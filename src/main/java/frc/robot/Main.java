// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * @brief The main file that houses program entry.
 */
public final class Main 
{
  /**
   * @brief Does Nothing...
   */
  private Main() 
  {
    // Intentionally Empty
  }

  /**
   * @brief The program entry point.
   * @param args Arguments.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
