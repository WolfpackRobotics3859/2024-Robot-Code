// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.constants.drivetrain.DriveConstants;

public class DriveAngle extends Command
{
  private final Supplier<Double> m_SpeedXSupplier, m_SpeedYSupplier, m_angleSupplier;
  private final Drivetrain m_Drivetrain;
  
  /**
   * Sends a field centric request to the given swerve drivetrain with given X speed, Y speed, and angle to point toward
   * @param drivetrain The swerve drivetrain object
   * @param speedX The speed in the X direction
   * @param speedY The speed in the Y direction
   * @param angle The angle to point the robot toward
   */
  public DriveAngle(Drivetrain drivetrain, Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> angle) 
  {
    this.m_SpeedXSupplier = speedX;
    this.m_SpeedYSupplier = speedY;
    this.m_angleSupplier = angle;
    this.m_Drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    // Intentionally Empty
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // create request
    final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(DriveConstants.MAX_SPEED * 0.1).withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(m_SpeedXSupplier.get() * DriveConstants.MAX_SPEED * 0.65)
      .withVelocityY(m_SpeedYSupplier.get() * DriveConstants.MAX_SPEED * 0.65)
      .withTargetDirection(Rotation2d.fromDegrees(m_angleSupplier.get()));

    // configure PID controller
    // TODO: tune these numbers
    driveRequest.HeadingController.setPID(0.01, 0.001, 0);
    driveRequest.HeadingController.setTolerance(0.5);

    m_Drivetrain.setControl(driveRequest);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // Intentionally Empty
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
