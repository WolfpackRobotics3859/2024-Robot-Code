// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SeedFieldRelative extends InstantCommand 
{
  private Drivetrain m_Drivetrain;
  
  /**
   * @brief Takes the current orientation of the robot and makes it X forward
   * @param drivetrain The swerve drivetrain object
   */
  public SeedFieldRelative(Drivetrain drivetrain) 
  {
    this.m_Drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Drivetrain.seedFieldRelative();
  }
}
