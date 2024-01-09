// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.constants.drivetrain.DrivetrainConstants;

public class drive extends Command
{
  private Supplier<Double> SPEED_X_SUPPLIER, SPEED_Y_SUPPLIER, ROTATIONAL_SPEED_SUPPLIER;
  private Drivetrain drivetrain = RobotContainer.drivetrain;
  
  
  public drive(Supplier<Double> SPEED_X_SUPPLIER, Supplier<Double> SPEED_Y_SUPPLIER, Supplier<Double> ROTATIONAL_SPEED_SUPPLIER) {
    this.SPEED_X_SUPPLIER = SPEED_X_SUPPLIER;
    this.SPEED_Y_SUPPLIER = SPEED_Y_SUPPLIER;
    this.ROTATIONAL_SPEED_SUPPLIER = ROTATIONAL_SPEED_SUPPLIER;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double SPEED_X = SPEED_X_SUPPLIER.get();
    double SPEED_Y = SPEED_Y_SUPPLIER.get();
    double ROTATIONAL_SPEED = ROTATIONAL_SPEED_SUPPLIER.get();

    final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1).withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withVelocityX(SPEED_X * DrivetrainConstants.MAX_SPEED)
      .withVelocityY(SPEED_Y * DrivetrainConstants.MAX_SPEED)
      .withRotationalRate(ROTATIONAL_SPEED * DrivetrainConstants.MAX_ANGULAR_RATE);

    drivetrain.setControl(driveRequest);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
