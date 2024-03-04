// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.constants.drivetrain.DrivetrainConstants;

public class DriveWithAngle extends Command
{
  private final Supplier<Double> m_SpeedXSupplier, m_SpeedYSupplier;
  private final double m_angle;
  private final Drivetrain m_Drivetrain;
  
  /**
   * Sends a field centric request to the given swerve drivetrain with given X speed, Y speed, and angle to point toward
   * @param drivetrain The swerve drivetrain object
   * @param speedX The speed in the X direction
   * @param speedY The speed in the Y direction
   * @param angle The angle to point the robot toward
   */
  public DriveWithAngle(Drivetrain drivetrain, Supplier<Double> speedX, Supplier<Double> speedY, Double angle) 
  {
    this.m_SpeedXSupplier = speedX;
    this.m_SpeedYSupplier = speedY;
    this.m_angle = angle;
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
    SwerveRequest.FieldCentricFacingAngle driveAngledRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(m_SpeedXSupplier.get() * DrivetrainConstants.MAX_SPEED * 0.65)
      .withVelocityY(m_SpeedYSupplier.get() * DrivetrainConstants.MAX_SPEED * 0.65)
      .withTargetDirection(Rotation2d.fromDegrees(m_angle));

    m_Drivetrain.setControl(driveAngledRequest);
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
