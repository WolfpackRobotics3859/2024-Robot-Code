// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot 
{
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() 
  {
    m_robotContainer = new RobotContainer();
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() 
  {
    // Intentionally Empty
  }

  @Override
  public void disabledPeriodic() 
  {
    // Intentionally Empty
  }

  @Override
  public void disabledExit() 
  {
    // Intentionally Empty
  }

  @Override
  public void autonomousInit() 
  {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() 
  {
    // Intentionally Empty
  }

  @Override
  public void autonomousExit() 
  {
    // Intentionally Empty
  }

  @Override
  public void teleopInit() 
  {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() 
  {
    // Intentionally Empty
  }

  @Override
  public void teleopExit() 
  {
    // Intentionally Empty
  }

  @Override
  public void testInit() 
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() 
  {
    // Intentionally Empty
  }

  @Override
  public void testExit() 
  {
    // Intentionally Empty
  }
}
