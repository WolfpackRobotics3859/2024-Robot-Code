// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

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
import frc.robot.commands.drivetrain.DriveWithTargetAngle;
import frc.robot.commands.drivetrain.SeedFieldRelative;
import frc.robot.commands.elevator.ElevatorPlayAlong;
import frc.robot.commands.elevator.KillElevator;
import frc.robot.commands.intake.IntakePlayAlong;
import frc.robot.commands.intake.KillIntake;
import frc.robot.commands.orchestrator.AmpShot;
import frc.robot.commands.orchestrator.BumperShot;
import frc.robot.commands.orchestrator.DefenseShot;
import frc.robot.commands.orchestrator.IWantANote;
import frc.robot.commands.orchestrator.ManualControl;
import frc.robot.commands.orchestrator.Stow;
import frc.robot.commands.shooter.KillShooter;
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
  private final CommandXboxController m_PrimaryController = new CommandXboxController(Hardware.PRIMARY_CONTROLLER_PORT);
  private final CommandXboxController m_SecondaryController = new CommandXboxController(Hardware.SECONDARY_CONTROLLER_PORT);
  Supplier<Double> m_PrimaryControllerLeftY = () -> -m_PrimaryController.getLeftY();
  Supplier<Double> m_PrimaryControllerLeftX = () -> -m_PrimaryController.getLeftX();
  Supplier<Double> m_PrimaryControllerRightX = () -> -m_PrimaryController.getRightX();

  /**
   * @brief Gets the Drivetrain subsystem.
   * @return The drivetrain object
   */
  public Drivetrain getDrive()
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
  public CommandXboxController getPrimaryController()
  {
    return this.m_PrimaryController;
  }

  public CommandXboxController getSecondaryController()
  {
    return this.m_SecondaryController;
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
  
    this.configureDefaultCommands();
    this.configureSmartDashboardCommands();
    this.configureBindings();
  }

  private void configureDefaultCommands()
  {
    m_Drivetrain.setDefaultCommand(new Drive(m_Drivetrain, m_PrimaryControllerLeftY, m_PrimaryControllerLeftX, m_PrimaryControllerRightX));
    m_Shooter.setDefaultCommand(new ShooterPlayAlong(m_Orchestrator, m_Shooter));
    m_Intake.setDefaultCommand(new IntakePlayAlong(m_Orchestrator, m_Intake));
    m_Elevator.setDefaultCommand(new ElevatorPlayAlong(m_Orchestrator, m_Elevator));
    m_Orchestrator.setDefaultCommand(new Stow(m_Orchestrator));
  }

  private void configureSmartDashboardCommands()
  {
    SmartDashboard.putData("Enable Manual Control", new ManualControl(m_Orchestrator));
    SmartDashboard.putData("Seed Odometry At Current Angle", new SeedFieldRelative(m_Drivetrain));
    SmartDashboard.putData("Kill All Subsystems (Excluding Drive)", new KillElevator(m_Elevator).alongWith(new KillIntake(m_Intake)).alongWith(new KillShooter(m_Shooter)));
    SmartDashboard.putData("Kill Shooter", new KillShooter(m_Shooter));
    SmartDashboard.putData("Kill Elevator", new KillElevator(m_Elevator));
    SmartDashboard.putData("Kill Intake", new KillIntake(m_Intake));
  }

  private void configureBindings() 
  {
    m_PrimaryController.leftTrigger().whileTrue(new IWantANote(m_Orchestrator));
    m_PrimaryController.rightTrigger().whileTrue(new BumperShot(m_Orchestrator));
    m_PrimaryController.rightBumper().whileTrue(new DefenseShot(m_Orchestrator));
    m_PrimaryController.leftBumper().whileTrue(new AmpShot(m_Orchestrator));
    m_PrimaryController.a().whileTrue(new DriveWithTargetAngle(m_Drivetrain, m_PrimaryControllerLeftY, m_PrimaryControllerLeftX, m_Drivetrain.YawToSpeaker));
    m_PrimaryController.y().onTrue(new SeedFieldRelative(m_Drivetrain));
  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}

