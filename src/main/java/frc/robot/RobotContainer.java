// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Hardware;
import frc.robot.constants.drivetrain.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.drivetrain.Drive;

public class RobotContainer 
{
  // Subsystems
  private final Drivetrain m_Drivetrain = new Drivetrain(TunerConstants.DRIVETRAIN_CONSTANTS, TunerConstants.FRONT_LEFT,
                      TunerConstants.FRONT_RIGHT, TunerConstants.BACK_LEFT, TunerConstants.BACK_RIGHT);
  private final Elevator m_Elevator = new Elevator();
  private final Shooter m_Shooter = new Shooter();
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();
  

  // Controllers
  private final CommandXboxController m_primaryController = new CommandXboxController(Hardware.PRIMARY_CONTROLLER_PORT);

  // Getters
  
  /**
   * @brief Gets the Drivetrain subsystem.
   * @return The drivetrain object
   */
  public Drivetrain getDriveSub()
  {
    return this.m_Drivetrain;
  }

  /**
   * @brief Gets the elevator subsystem.
   * @return The elevator object
   */
  public Elevator getElevator()
  {
    return this.m_Elevator;
  }

  /**
   * @brief Gets the shooter subsystem.
   * @return The shooter object
   */
  public Shooter getShooter()
  {
    return this.m_Shooter;
  }

   /**
   * @brief Gets the intake subsystem.
   * @return The intake object
   */
  public IntakeSubsystem getIntake()
  {
    return this.m_Intake;
  }

  public RobotContainer() 
  {
    configureBindings();
  }
  
  private void configureBindings() 
  {
    m_Drivetrain.setDefaultCommand(
      new Drive(m_Drivetrain,
                () -> -m_primaryController.getLeftY(),
                () -> -m_primaryController.getLeftX(),
                () -> -m_primaryController.getRightX()
      ));
  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}
