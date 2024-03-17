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

public class DriveWithTargetAngle extends Command
{
  private final Supplier<Double> m_SpeedXSupplier, m_SpeedYSupplier;
  private final Supplier<Rotation2d> m_AngleSupplier;
  private final Drivetrain m_Drivetrain;
  private final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle();
  
  /**
   * Sends a field centric request to the given swerve drivetrain with given X speed, Y speed, and angle to point toward
   * @param drivetrain The swerve drivetrain object
   * @param speedX The speed in the X direction
   * @param speedY The speed in the Y direction
   * @param angle The angle to point the robot toward
   */
  public DriveWithTargetAngle(Drivetrain drivetrain, Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Rotation2d> angle) 
  {
    this.m_SpeedXSupplier = speedX;
    this.m_SpeedYSupplier = speedY;
    this.m_AngleSupplier = angle;
    this.m_Drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    driveRequest.HeadingController.setPID
    (
      DriveConstants.TURN_TO_ANGLE_P,
      DriveConstants.TURN_TO_ANGLE_I,
      DriveConstants.TURN_TO_ANGLE_D
    );
    driveRequest.HeadingController.setTolerance(DriveConstants.TURN_TO_ANGLE_TOLERANCE);
    driveRequest.HeadingController.enableContinuousInput(Math.PI, -Math.PI);

    m_Drivetrain.setAligned(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // modify request
    driveRequest
      .withDeadband(DriveConstants.MAX_SPEED * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(m_SpeedXSupplier.get() * DriveConstants.MAX_SPEED * 0.65)
      .withVelocityY(m_SpeedYSupplier.get() * DriveConstants.MAX_SPEED * 0.65)
      .withTargetDirection(m_AngleSupplier.get());

    m_Drivetrain.setControl(driveRequest);

    if(driveRequest.HeadingController.atSetpoint())
    {
      m_Drivetrain.setAligned(true);
    }
    else
    {
      m_Drivetrain.setAligned(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_Drivetrain.setAligned(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
