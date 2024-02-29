// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.drivetrain.DrivetrainConstants;
import frc.robot.constants.drivetrain.TunerConstants;

public class Drivetrain extends SwerveDrivetrain implements Subsystem 
{
  private boolean m_odometrySeeded = false;
  private PhotonCamera m_photonCamera;
  private PhotonPoseEstimator m_photonPoseEstimator;
  private Timer m_timer;

  private final SwerveRequest.ApplyChassisSpeeds m_autoRequest = new SwerveRequest.ApplyChassisSpeeds();

  /** 
    @brief Configure PathPlanner objects for automatic path following
  */
  private void configurePathPlanner()
  {
    //Determine the radius of the drivebase from module locations
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) 
    {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }
    
    //Create drivetrain object for pathplanner to use in its calculations
    AutoBuilder.configureHolonomic(
      ()->this.getState().Pose,
      this::seedFieldRelative,
      this::getCurrentRobotChassisSpeeds,
      (speeds)->this.setControl(m_autoRequest.withSpeeds(speeds)),
      new HolonomicPathFollowerConfig(new PIDConstants(7, 0, 0), new PIDConstants(7, 0, 0), TunerConstants.SPEED_AT_12_VOLTS_MPS, driveBaseRadius, new ReplanningConfig()),
      ()->false,
      this);

  }

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
    // Create a photon camera and pose estimator object
    m_photonCamera = new PhotonCamera("front_camera");
    m_photonPoseEstimator = new PhotonPoseEstimator(DrivetrainConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_photonCamera, DrivetrainConstants.FORWARD_CAMERA_POSITION);

    // Create a timer for less critical tasks such as Smartdashboard updates
    this.m_timer = new Timer();
    m_timer.start();
    configurePathPlanner();
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

  public SwerveDrivePoseEstimator getOdometry() {
    return m_odometry;
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds()
  {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  @Override
  public void periodic() 
  {
    //Publish pose for advantagescope odometry
    Logger.recordOutput("robotPose", m_odometry.getEstimatedPosition());

    // Ask Photon for a generated pose
    Optional<EstimatedRobotPose> estPose = m_photonPoseEstimator.update();

    // Checks if Photon returned a pose
    if (!estPose.isEmpty() && m_photonCamera.getLatestResult().getBestTarget().getPoseAmbiguity() < DrivetrainConstants.AMBIGUITY_THRESHOLD)
    { 
      // Seeds an initial odometry value from vision system
      if (!m_odometrySeeded)
      {
        // Seed pose
        this.m_odometry.resetPosition(estPose.get().estimatedPose.getRotation().toRotation2d(), m_modulePositions,
            estPose.get().estimatedPose.toPose2d());
        m_odometrySeeded = true;
      } 
      else
      {
        // Add vision to kalman filter
        this.addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), estPose.get().timestampSeconds);
        SmartDashboard.putString("Pose - Vision", estPose.get().estimatedPose.toPose2d().toString());
      }
    }
    // Report robots current pose to smartdashboard every half second
    if (m_timer.get() > 0.5)
    {
      m_timer.reset();
      SmartDashboard.putString("Pose - Drivetrain", m_odometry.getEstimatedPosition().toString());
      try 
      {
        SmartDashboard.putNumber("Pose Ambuguity", m_photonCamera.getLatestResult().getBestTarget().getPoseAmbiguity());
      } 
      catch(Exception e) 
      {
        //Intenionally Empty
      }
      SmartDashboard.putBoolean("Odometry seeded", m_odometrySeeded);
    }
  }
}
