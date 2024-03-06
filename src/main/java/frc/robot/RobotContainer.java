// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Hardware;
import frc.robot.constants.drivetrain.TunerConstants;
import frc.robot.constants.elevator.ElevatorConstants;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.DriveWithAngle;
import frc.robot.commands.drivetrain.SeedFieldRelative;
import frc.robot.commands.elevator.ElevatorPlayAlong;
import frc.robot.commands.intake.IntakePlayAlong;
import frc.robot.commands.orchestrator.ManualControl;
import frc.robot.commands.shooter.ShooterPlayAlong;

public class RobotContainer 
{
  // Subsystems
  private final Drivetrain m_Drivetrain = new Drivetrain(TunerConstants.DRIVETRAIN_CONSTANTS, 250, TunerConstants.FRONT_LEFT,
                      TunerConstants.FRONT_RIGHT, TunerConstants.BACK_LEFT, TunerConstants.BACK_RIGHT);
  private final Elevator m_Elevator = new Elevator();
  private final Shooter m_Shooter = new Shooter();
  private final Intake m_Intake = new Intake();

  // Orchestrator
  private final Orchestrator m_Orchestrator = new Orchestrator(m_Drivetrain, m_Elevator, m_Shooter, m_Intake);

  // Controllers
  private final CommandXboxController m_primaryController = new CommandXboxController(Hardware.PRIMARY_CONTROLLER_PORT);
  private final CommandXboxController m_secondaryController = new CommandXboxController(Hardware.SECONDARY_CONTROLLER_PORT);

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
  public Intake getIntake()
  {
    return this.m_Intake;
  }

  public Orchestrator getOrchestrator()
  {
    return this.m_Orchestrator;
  }
  
  /**
   * Gets the driver controller
   * @return The primary (driver) controller
   */
  public CommandXboxController getController()
  {
    return this.m_primaryController;
  }

  public RobotContainer() 
  {
    configureBindings();

    SmartDashboard.putNumber("Amp Shot Wrist Position", ShooterConstants.WRIST_AMP_SHOOTING_POSITION);
    SmartDashboard.putNumber("Amp Shot Elevator Position", ElevatorConstants.ELEVATOR_AMP_SHOT_POSITION);
    SmartDashboard.putNumber("Amp Shot Motor 1 Velocity", 5);
    SmartDashboard.putNumber("Amp Shot Motor 2 Velocity", 17.5);

    SmartDashboard.setDefaultNumber("Amp Shot Wrist Position", ShooterConstants.WRIST_AMP_SHOOTING_POSITION);
    SmartDashboard.setDefaultNumber("Amp Shot Elevator Position", ElevatorConstants.ELEVATOR_AMP_SHOT_POSITION);
    SmartDashboard.setDefaultNumber("Amp Shot Motor 1 Velocity", 6);
    SmartDashboard.setDefaultNumber("Amp Shot Motor 1 Velocity", 17.5);

  }

  
  private void configureBindings() 
  {
    m_Drivetrain.setDefaultCommand(
      new Drive(m_Drivetrain,
                () -> -m_primaryController.getLeftY(),
                () -> -m_primaryController.getLeftX(),
                () -> -m_primaryController.getRightX()
      ));

    m_Shooter.setDefaultCommand(new ShooterPlayAlong(m_Orchestrator, m_Shooter));
    m_Intake.setDefaultCommand(new IntakePlayAlong(m_Orchestrator, m_Intake));
    m_Elevator.setDefaultCommand(new ElevatorPlayAlong(m_Orchestrator, m_Elevator));
    m_Orchestrator.setDefaultCommand(new ManualControl(m_Orchestrator));

    m_primaryController.a().whileTrue(new DriveWithAngle(m_Drivetrain,
        () -> -m_primaryController.getLeftY(),
        () -> -m_primaryController.getLeftX(),
        180.0));

    m_primaryController.b().whileTrue(new DriveWithAngle(m_Drivetrain,
        () -> -m_primaryController.getLeftY(),
        () -> -m_primaryController.getLeftX(),
        90.0));

    m_primaryController.y().whileTrue(new DriveWithAngle(m_Drivetrain,
        () -> -m_primaryController.getLeftY(),
        () -> -m_primaryController.getLeftX(),
        0.0));

    m_primaryController.x().whileTrue(new DriveWithAngle(m_Drivetrain,
        () -> -m_primaryController.getLeftY(),
        () -> -m_primaryController.getLeftX(),
        270.0));

    m_secondaryController.x().whileTrue(new DriveWithAngle(m_Drivetrain, // left chain
      () -> -m_primaryController.getLeftY(),
      () -> -m_primaryController.getLeftX(),
      120.0)); // change angle at some point

    m_secondaryController.b().whileTrue(new DriveWithAngle(m_Drivetrain, // right chain
      () -> -m_primaryController.getLeftY(),
      () -> -m_primaryController.getLeftX(),
      300.0)); // change angle at some point

    m_secondaryController.y().whileTrue(new DriveWithAngle(m_Drivetrain, // back chain
      () -> -m_primaryController.getLeftY(),
      () -> -m_primaryController.getLeftX(),
      180.0)); // change angle at some point

    m_secondaryController.a().whileTrue(new SeedFieldRelative(m_Drivetrain));
  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}

