// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.drivetrain.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.SeedFieldRelative;

public class RobotContainer 
{
  // Subsystems
  private final Drivetrain m_Drivetrain = new Drivetrain(TunerConstants.DRIVETRAIN_CONSTANTS, TunerConstants.FRONT_LEFT,
                      TunerConstants.FRONT_RIGHT, TunerConstants.BACK_LEFT, TunerConstants.BACK_RIGHT);
  private final Shooter m_Shooter = new Shooter();
  
  // Controllers
  private final CommandXboxController primaryController = new CommandXboxController(0);

  public Drivetrain getDriveSub()
  {
    return this.m_Drivetrain;
  }
  
  private void configureBindings() 
  {
    m_Drivetrain.setDefaultCommand(
      new Drive(m_Drivetrain,
                () -> -primaryController.getLeftY(),
                () -> -primaryController.getLeftX(),
                () -> -primaryController.getRightX()
      ));

    primaryController.a().onTrue(new SeedFieldRelative(m_Drivetrain));
  }

  public RobotContainer() 
  {
    configureBindings();
  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}
