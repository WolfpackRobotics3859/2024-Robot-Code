// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.commands.autos.DriveBack;
import frc.robot.commands.autos.LongDriveBack;
import frc.robot.commands.autos.Shoot;
import frc.robot.commands.autos.ShootAndDrive;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.DriveWithAngle;
import frc.robot.commands.drivetrain.SeedFieldRelative;
import frc.robot.commands.elevator.ElevatorPlayAlong;
import frc.robot.commands.intake.IntakePlayAlong;
import frc.robot.commands.orchestrator.AmpShot;
import frc.robot.commands.orchestrator.BumperShot;
import frc.robot.commands.orchestrator.Climb;
import frc.robot.commands.orchestrator.IntakeCommand;
import frc.robot.commands.orchestrator.Purge;
import frc.robot.commands.orchestrator.Stow;
import frc.robot.commands.orchestrator.ZeroIntake;
import frc.robot.commands.shooter.ShooterPlayAlong;

public class RobotContainer 
{
  // Subsystems
  private final Drivetrain m_Drivetrain = new Drivetrain(TunerConstants.DRIVETRAIN_CONSTANTS, TunerConstants.FRONT_LEFT,
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

  private final SendableChooser<Command> autoSelector = new SendableChooser<>();
  private final SendableChooser<Double> angleSelector = new SendableChooser<>();

  public RobotContainer() 
  {
    // seed angle chooser
    angleSelector.setDefaultOption("Front", 180.0);
    angleSelector.addOption("Left", 300.0);
    angleSelector.addOption("Right", 240.0);

    autoSelector.setDefaultOption("Drive Back", new DriveBack(m_Orchestrator, m_Drivetrain, angleSelector.getSelected()));
    autoSelector.addOption("Shoot", new Shoot(m_Orchestrator, m_Drivetrain, angleSelector.getSelected()));
    autoSelector.addOption("Shoot and Drive Back", new ShootAndDrive(m_Orchestrator, m_Drivetrain, angleSelector.getSelected()));
    autoSelector.addOption("Long Drive Back", new LongDriveBack(m_Orchestrator, m_Drivetrain, angleSelector.getSelected()));

    SmartDashboard.putData(autoSelector);
    SmartDashboard.putData(angleSelector);
           
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

    // default command
    m_Orchestrator.setDefaultCommand(new Stow(m_Orchestrator));

    //TODO these should be changed to be able to be killed later for climb or errors
    m_Shooter.setDefaultCommand(new ShooterPlayAlong(m_Orchestrator, m_Shooter));
    m_Intake.setDefaultCommand(new IntakePlayAlong(m_Orchestrator, m_Intake));
    m_Elevator.setDefaultCommand(new ElevatorPlayAlong(m_Orchestrator, m_Elevator, () -> -m_secondaryController.getRightY()));

    // PRIMARY CONTROLLER
    m_primaryController.leftTrigger().whileTrue(new IntakeCommand(m_Orchestrator).unless(() -> m_Orchestrator.noteStowed));
    m_primaryController.rightTrigger().whileTrue(new BumperShot(m_Orchestrator, false).unless(() -> !m_Orchestrator.noteStowed));
    m_primaryController.rightBumper().whileTrue(new AmpShot(m_Orchestrator));
    m_primaryController.leftBumper().onTrue(new SeedFieldRelative(m_Drivetrain));
    
    m_primaryController.a().whileTrue(new DriveWithAngle(m_Drivetrain,
        () -> -m_primaryController.getLeftY(),
        () -> -m_primaryController.getLeftX(),
        180.0));

    m_primaryController.y().whileTrue(new DriveWithAngle(m_Drivetrain,
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

    // SECONDARY CONTROLLER
    // m_secondaryController.leftBumper() UNDER DEFENSE
    m_secondaryController.leftTrigger().whileTrue(new Purge(m_Orchestrator)); // purge
    m_secondaryController.rightTrigger().whileTrue(new Climb(m_Orchestrator)); // start climb
    m_secondaryController.leftBumper().whileTrue(new ZeroIntake(m_Intake)); // zero intake

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

    //TODO fix these
    // m_secondaryController.b().whileTrue(new Drive(m_Drivetrain, // jog robot forward
    //   () -> -m_primaryController.getLeftY(),
    //   () -> 0.0,
    //   () -> 0.0
    // ));

    // m_secondaryController.leftBumper().whileTrue(new PathPlannerAuto("Amp Auto"));
  }

  public Command getAutonomousCommand() 
  {
    return autoSelector.getSelected();
  }
}

