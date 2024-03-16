// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Hardware;
import frc.robot.constants.drivetrain.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.autos.AmpShotAuto;
import frc.robot.commands.autos.IntakeAuto;
import frc.robot.commands.autos.LowShotAuto;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.DriveWithTargetAngle;
import frc.robot.commands.drivetrain.SeedFieldRelative;
import frc.robot.commands.elevator.ElevatorPlayAlong;
import frc.robot.commands.elevator.KillElevator;
import frc.robot.commands.intake.IntakePlayAlong;
import frc.robot.commands.intake.KillIntake;
import frc.robot.commands.orchestrator.AmpPrep;
import frc.robot.commands.orchestrator.AmpShot;
import frc.robot.commands.orchestrator.Climb;
import frc.robot.commands.orchestrator.ClimbPrep;
import frc.robot.commands.orchestrator.DefenseShot;
import frc.robot.commands.orchestrator.DisableVision;
import frc.robot.commands.orchestrator.IntakeCommand;
import frc.robot.commands.orchestrator.LowShot;
import frc.robot.commands.orchestrator.ManualControl;
import frc.robot.commands.orchestrator.ShootAmp;
import frc.robot.commands.orchestrator.Stow;
import frc.robot.commands.orchestrator.ZeroIntake;
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
  private final Supplier<Double> m_PrimaryControllerLeftY = () -> -m_PrimaryController.getLeftY();
  private final Supplier<Double> m_PrimaryControllerLeftX = () -> -m_PrimaryController.getLeftX();
  private final Supplier<Double> m_PrimaryControllerRightX = () -> -m_PrimaryController.getRightX();
  private final Supplier<Double> m_SecondaryControllerRightY = () -> -m_SecondaryController.getRightY();

  // Choosers
  private final SendableChooser<Command> autoSelector = new SendableChooser<>();
  private final SendableChooser<Double> seedAngleSelector = new SendableChooser<>();

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
    this.configureDefaultCommands();
    this.configureSmartDashboardCommands();
    this.configureAutoSelector();
    this.configureBindings();
    this.configureAutoCommands();

    SmartDashboard.putData("Auto Selector", autoSelector);
  }

  private void configureDefaultCommands()
  {
    m_Drivetrain.setDefaultCommand(new Drive(m_Drivetrain, m_PrimaryControllerLeftY, m_PrimaryControllerLeftX, m_PrimaryControllerRightX));
    m_Shooter.setDefaultCommand(new ShooterPlayAlong(m_Orchestrator, m_Shooter));
    m_Intake.setDefaultCommand(new IntakePlayAlong(m_Orchestrator, m_Intake));
    m_Elevator.setDefaultCommand(new ElevatorPlayAlong(m_Orchestrator, m_Elevator));
    m_Orchestrator.setDefaultCommand(new Stow(m_Orchestrator));
  }

  private void configureAutoCommands()
  {
    NamedCommands.registerCommand("AmpShotAuto", new AmpShotAuto(m_Orchestrator, m_Shooter));
    NamedCommands.registerCommand("LowShotAuto", new LowShotAuto(m_Orchestrator, m_Shooter, m_Drivetrain));
    NamedCommands.registerCommand("IntakeAuto", new IntakeAuto(m_Orchestrator, m_Shooter));
  }

  private void configureSmartDashboardCommands()
  {
    SmartDashboard.putData("Enable Manual Control", new ManualControl(m_Orchestrator));
    SmartDashboard.putData("Kill All Subsystems (Excluding Drive)", new KillElevator(m_Elevator).alongWith(new KillIntake(m_Intake)).alongWith(new KillShooter(m_Shooter)));
    SmartDashboard.putData("Kill Shooter", new KillShooter(m_Shooter));
    SmartDashboard.putData("Kill Elevator", new KillElevator(m_Elevator));
    SmartDashboard.putData("Kill Intake", new KillIntake(m_Intake));
    SmartDashboard.putData("Disable Vision", new DisableVision(m_Drivetrain));
  }

  private void configureAutoSelector()
  {
    // TODO: add auto options
    autoSelector.setDefaultOption("None", new PrintCommand("No auto selected."));
    autoSelector.addOption("Shoot + Nun", new PathPlannerAuto("Shoot + Nun"));

    // Seed Angle Selector
    seedAngleSelector.setDefaultOption("Center", 180.0);
    seedAngleSelector.addOption("Left", 120.0);
    seedAngleSelector.addOption("Right", 240.0);
  }


  private void configureBindings() 
  {
    // PRIMARY CONTROLLER
    m_PrimaryController.rightTrigger().whileTrue(new IntakeCommand(m_Orchestrator)); // intake
    m_PrimaryController.leftTrigger().whileTrue(new ParallelCommandGroup // low shot
    (
      new LowShot(m_Orchestrator),
      new DriveWithTargetAngle(m_Drivetrain, m_PrimaryControllerLeftY, m_PrimaryControllerLeftX, m_Drivetrain.yawToSpeaker)
    ));
    // m_PrimaryController.leftBumper().whileTrue(new AmpShot(m_Orchestrator)); // amp shot
    m_PrimaryController.rightBumper().whileTrue(new DefenseShot(m_Orchestrator)); // defense shot
    m_PrimaryController.b().onTrue(new SeedFieldRelative(m_Drivetrain)); // reset gyro
    m_PrimaryController.leftBumper().whileTrue(new ShootAmp(m_Orchestrator)); // shoot amp after prep (preps if not yet)

    // SECONDARY CONTROLLER
    m_SecondaryController.rightTrigger().whileTrue(new ClimbPrep(m_Orchestrator)); // move to climb
    m_SecondaryController.leftTrigger().whileTrue // climb
    (
      new ParallelCommandGroup
      (
        new Climb(m_Elevator, m_SecondaryControllerRightY),
        new WaitUntilCommand(m_Elevator.killShooterForClimb).andThen(new KillShooter(m_Shooter))
      )
    );
    m_SecondaryController.y().whileTrue(new ZeroIntake(m_Intake)); // zero intake
    m_SecondaryController.rightBumper().whileTrue(new AmpPrep(m_Orchestrator)); // prepare amp
  }

  public Command getAutonomousCommand() 
  {
    return autoSelector.getSelected();
  }
}

