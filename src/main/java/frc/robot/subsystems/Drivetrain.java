// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Drivetrain extends SwerveDrivetrain implements Subsystem 
{
  private boolean odometrySeeded = false;
  private PhotonCamera m_photonCamera;
  private PhotonPoseEstimator m_photonPoseEstimator;
  private Pose3d currentPose;
  private Timer timer;
  
  /** 
    @brief Creates a new Drivetrain.
    @param driveTrainConstants Drivetrain-wide constants for the swerve drive
    @param OdometryUpdateFrequency The frequency to run the odometry loop. If unspecified, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0
    @param modules Constants for each specific module 
  */
  public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules)
  {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    
  }

  /** 
    @brief Creates a new Drivetrain without specifying the frequency to run the odometry loop.
    @param driveTrainConstants Drivetrain-wide constants for the swerve drive
    @param modules Constants for each specific module 
  */
  public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules)
  {
    super(driveTrainConstants, modules);
    m_photonCamera = new PhotonCamera("photonvision");
    m_photonPoseEstimator = new PhotonPoseEstimator(Constants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_photonCamera, Constants.ROBOT_TO_CAM); //8.5 inches back from center, directly centered, 33in tall
    currentPose = new Pose3d(); 
    this.timer = new Timer();
    timer.start();
  }


  /**
   * @brief Applies a swerve request to the drivetrain
   * @param requestSupplier Supplier for the swerve request
   * @return Applies the given swerve request to the swerve modules
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier)
  {
    return run(() -> this.setControl(requestSupplier.get()));  
  }


  @Override
  public void periodic() 
  {
    Optional<EstimatedRobotPose> estPose = m_photonPoseEstimator.update();
    //Seeds an initial odometry value from vision system
    if(!odometrySeeded) {
      //Update vision data
      //Check if pose data is valid
      if(!estPose.isEmpty()) 
      {
        //Seed pose
        this.m_odometry.resetPosition(estPose.get().estimatedPose.getRotation().toRotation2d(), m_modulePositions, estPose.get().estimatedPose.toPose2d());
        //this.currentPose = estPose.get().estimatedPose;
        odometrySeeded = true;
      }
    } else {
      //Check if pose data is valid
      if(!estPose.isEmpty()) {      
        //Add vision to kalman filter
        addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), ModuleCount);
        //Bad
        currentPose = estPose.get().estimatedPose;
      }

    }
    //Report robots current pose to smartdashboard every second
    if(timer.get() > 1.0) 
    {
      timer.reset();
      System.out.println("Timer tick");
      SmartDashboard.putString("Pose - Vision", currentPose.toPose2d().toString());
      SmartDashboard.putString("Pose - Drivetrain", m_odometry.getEstimatedPosition().toString());

      SmartDashboard.putBoolean("Od seeded", odometrySeeded);
    }
  }
}
