// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * @brief The object that declares button bindings and chooses autonomous commands.
 */
public class RobotContainer 
{
  private RoboLogger m_RoboLogger;

  /**
   * @brief Creates a RobotContainer.
   * @param logger A RoboLogger object.
   */
  public RobotContainer(RoboLogger logger) 
  {
    m_RoboLogger = logger;
    configureBindings();
  }

  /**
   * @brief Configures the controller bindings.
   */
  private void configureBindings()
  {
    // Intentionally Empty
  }

  /**
   * @brief Chooses an autonomous commands.
   * @return The chosen autonomous command.
   */
  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured.");
  }
}
