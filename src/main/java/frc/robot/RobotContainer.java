// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.drivetrain.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.constants.drivetrain.DrivetrainConstants;

public class RobotContainer {

  public static final Drivetrain DriveTrain = new Drivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft,
                      TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

  private final CommandXboxController m_operatorController = new CommandXboxController(0);
  public final Drivetrain drivetrain = DriveTrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1).withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private void configureBindings() {
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> drive.withVelocityX(-m_operatorController.getLeftY() * DrivetrainConstants.MAX_SPEED)
      .withVelocityY(-m_operatorController.getLeftX() * DrivetrainConstants.MAX_SPEED)
      .withRotationalRate(-m_operatorController.getRightX() * DrivetrainConstants.MAX_ANGULAR_RATE)
    ));

    m_operatorController.a().whileTrue(drivetrain.applyRequest(() -> brake));

    m_operatorController.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
  }

  public RobotContainer() {
    configureBindings();
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
