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
import frc.robot.commands.vision.EnableVision;

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

    this.configureDefaultCommands();
    this.configureAutoSelector();
    this.configureBindings();

    SmartDashboard.putData("Auto Selector", autoSelector);
    SmartDashboard.putData("Disable Vision", new DisableVision(m_Vision));
    SmartDashboard.putData("Enable Vision", new EnableVision(m_Vision));
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

  private void registerCommands()
  {
    // Intentionally Empty
  }



  private void configureAutoSelector()
  {
    autoSelector.setDefaultOption("None", new SeedFieldRelative(m_Drivetrain));
    autoSelector.addOption("2 Note From Source", new PathPlannerAuto("2NoteFromSource"));
    autoSelector.addOption("4 Note From Amp", new PathPlannerAuto("4NoteFromAmp"));
    autoSelector.addOption("4 Note From Amp (Close)", new PathPlannerAuto("4NoteFromAmpClose"));
    autoSelector.addOption("3 Note From Amp", new PathPlannerAuto("3NoteFromAmp"));
  }

  private void configureBindings() 
  {
    /* 
      primary:
      right trigger = intake
      left trigger = shot
      right bumper = unassigned
      left bumper = amp
      y = reset gyro

      
      secondary:
      right bumper = reset intake
      x = purge
    */ 
  }

  public Command getAutonomousCommand() 
  {
    return autoSelector.getSelected();
  }
}

