// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  /** 
    @brief Creates a new Drivetrain.
    @param driveTrainConstants Drivetrain-wide constants for the swerve drive
    @param OdometryUpdateFrequency The frequency to run the odometry loop. If unspecified, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0
    @param modules Constants for each specific module 
  */
  public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
    }

  /** 
    @brief Creates a new Drivetrain without specifying the frequency to run the odometry loop.
    @param driveTrainConstants Drivetrain-wide constants for the swerve drive
    @param modules Constants for each specific module 
  */
  public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
    }

  /**
   * @brief Applies a swerve request to the drivetrain
   * @param requestSupplier Supplier for the swerve request
   * @return Applies the given swerve request to the swerve modules
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));  
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
