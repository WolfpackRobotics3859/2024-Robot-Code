// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.constants.drivetrain.DrivetrainConstants;

public class Drive extends Command
{
  /*
   * speedXSupplier - The supplier object for speed in the X direction
   * speedYSupplier - The supplier object for speed in the Y direction
   * rotationalSpeedSupplier - The supplier object for rotational speed
   */
  private Supplier<Double> speedXSupplier, speedYSupplier, rotationalSpeedSupplier;
  private Drivetrain m_Drivetrain;
  
  /**
   * @brief Sends a field centric request to the given swerve drivetrain with given X speed, Y speed, and rotational speed
   * @param drivetrain The swerve drivetrain object
   * @param speedX The speed in the X direction
   * @param speedY The speed in the Y direction
   * @param rotationalSpeed The rotational speed
   */
  public Drive(Drivetrain drivetrain, Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> rotationalSpeed) 
  {
    this.speedXSupplier = speedX;
    this.speedYSupplier = speedY;
    this.rotationalSpeedSupplier = rotationalSpeed;
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
    final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1).withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withVelocityX(speedXSupplier.get() * DrivetrainConstants.MAX_SPEED)
      .withVelocityY(speedYSupplier.get() * DrivetrainConstants.MAX_SPEED)
      .withRotationalRate(rotationalSpeedSupplier.get() * DrivetrainConstants.MAX_ANGULAR_RATE);

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
  public boolean isFinished() {
    return false;
  }
}
