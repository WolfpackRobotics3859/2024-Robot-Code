// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.Hardware;
import frc.robot.constants.drivetrain.TunerConstants;
import frc.robot.constants.vision.VisionConstants;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Vision;

import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.DriveWithTargetAngle;
import frc.robot.commands.drivetrain.SeedFieldRelative;
import frc.robot.commands.vision.DisableVision;

public class RobotContainer 
{
  // Vision
  private final Vision m_Vision = new Vision(VisionConstants.CAMERAS);
  
  // Subsystems
  private final Drivetrain m_Drivetrain = new Drivetrain(m_Vision, TunerConstants.DRIVETRAIN_CONSTANTS, 250, TunerConstants.FRONT_LEFT,
                                                         TunerConstants.FRONT_RIGHT, TunerConstants.BACK_LEFT, TunerConstants.BACK_RIGHT);
  private final Elevator m_Elevator = new Elevator();
  private final Shooter m_Shooter = new Shooter();
  private final Intake m_Intake = new Intake();

  // Controllers
  private final CommandXboxController m_PrimaryController = new CommandXboxController(Hardware.PRIMARY_CONTROLLER_PORT);
  private final CommandXboxController m_SecondaryController = new CommandXboxController(Hardware.SECONDARY_CONTROLLER_PORT);

  // Controller Suppliers
  private final Supplier<Double> m_PrimaryControllerLeftY = () -> -m_PrimaryController.getLeftY() * m_Drivetrain.axisModifier;
  private final Supplier<Double> m_PrimaryControllerLeftX = () -> -m_PrimaryController.getLeftX() * m_Drivetrain.axisModifier;
  private final Supplier<Double> m_PrimaryControllerRightX = () -> -m_PrimaryController.getRightX();

  // Auto Chooser
  private final SendableChooser<Command> autoSelector = new SendableChooser<>();

  public RobotContainer() 
  {  
    this.configureAutoCommands();
    this.configureDefaultCommands();
    this.configureSmartDashboardCommands();
    this.configureAutoSelector();
    this.configureBindings();

    SmartDashboard.putData("Auto Selector", autoSelector);
  }

  private void configureDefaultCommands()
  {
    m_Drivetrain.setDefaultCommand
    (
      new Drive
      (
        m_Drivetrain,
        m_PrimaryControllerLeftY, 
        m_PrimaryControllerLeftX, 
        m_PrimaryControllerRightX
      )
    );


  }

  private void configureAutoCommands()
  {
  }

  private void configureSmartDashboardCommands()
  {
  }

  private void configureAutoSelector()
  {
    // TODO: add auto options
    autoSelector.setDefaultOption("None", new SeedFieldRelative(m_Drivetrain));
    autoSelector.addOption("2 Note From Source", new PathPlannerAuto("2NoteFromSource"));
    autoSelector.addOption("4 Note From Amp", new PathPlannerAuto("4NoteFromAmp"));
    autoSelector.addOption("4 Note From Amp (Close)", new PathPlannerAuto("4NoteFromAmpClose"));
    autoSelector.addOption("3 Note From Amp", new PathPlannerAuto("3NoteFromAmp"));
  }

  private void configureBindings() 
  {
  //   // PRIMARY CONTROLLER
  //   m_PrimaryController.rightTrigger().whileTrue(new IntakeCommand(m_Orchestrator)); // intake
  //   m_PrimaryController.leftTrigger().whileTrue(new ConditionalCommand // low shot
  //   (
  //     new ParallelCommandGroup(
  //       new LowShot(m_Orchestrator),
  //       new DriveWithTargetAngle(m_Drivetrain, m_PrimaryControllerLeftY, m_PrimaryControllerLeftX, m_Drivetrain.yawToSpeaker)
  //     ),
  //     new LowShot(m_Orchestrator),
  //     () -> m_Drivetrain.getVisionEnabled()
  //   ));
  //   m_PrimaryController.rightBumper().whileTrue(new DefenseShot(m_Orchestrator)); // defense shot
  //   m_PrimaryController.leftBumper().whileTrue(new ParallelCommandGroup // shoot amp after prep (preps if not yet)
  //   (
  //     new AmpPrep(m_Orchestrator),
  //     new ShootAmp(m_Orchestrator)
  //   ));

  //   // SECONDARY CONTROLLER
  //   m_SecondaryController.rightTrigger().whileTrue(new ClimbPrep(m_Orchestrator)); // move to climb
  //   m_SecondaryController.leftTrigger().whileTrue // climb
  //   (
  //     new ParallelCommandGroup
  //     (
  //       new Climb(m_Elevator, m_SecondaryControllerRightY),
  //       new WaitUntilCommand(m_Elevator.killShooterForClimb).andThen(new KillShooter(m_Shooter))
  //     )
  //   );
  //   m_SecondaryController.y().whileTrue(new ZeroIntake(m_Intake)); // zero intake
  //   m_SecondaryController.leftBumper().whileTrue(new AmpPrep(m_Orchestrator)); // prepare amp
  //   m_SecondaryController.x().whileTrue(new Purge(m_Orchestrator)); // purge
  //   m_SecondaryController.a().whileTrue(new LowShot(m_Orchestrator));
  //   m_SecondaryController.rightBumper().onTrue(new SeedFieldRelative(m_Drivetrain));
  }

  public Command getAutonomousCommand() 
  {
    return autoSelector.getSelected();
  }
}

