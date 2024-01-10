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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.drivetrain.DrivetrainConstants;

public class Drivetrain extends SwerveDrivetrain implements Subsystem
{
  private boolean m_odometrySeeded = false;
  private PhotonCamera m_photonCamera;
  private PhotonPoseEstimator m_photonPoseEstimator;
  private Timer m_timer;
  StructPublisher<Pose2d> m_posePublisher = NetworkTableInstance.getDefault().getStructTopic("robotPose", Pose2d.struct).publish();
  /**
   * @brief Creates a new Drivetrain.
   * @param driveTrainConstants Drivetrain-wide constants for the swerve drive
   * @param OdometryUpdateFrequency The frequency to run the odometry loop. If
   * unspecified, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0
   * @param modules Constants for each specific module
   */
  public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
      SwerveModuleConstants... modules)
  {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
  }

  /**
   * @brief Creates a new Drivetrain without specifying the frequency to run the
   * odometry loop.
   * @param driveTrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  /**
   * @brief Creates a new Drivetrain without specifying the frequency to run the
   * odometry loop.
   * @param driveTrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules)
  {
    super(driveTrainConstants, modules);
    // Create a photon camera and pose estimator object
    // Create a photon camera and pose estimator object
    m_photonCamera = new PhotonCamera("photonvision");
    m_photonPoseEstimator = new PhotonPoseEstimator(DrivetrainConstants.TAG_LAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_photonCamera, DrivetrainConstants.ROBOT_TO_CAM);

    // Create a timer for less critical tasks such as Smartdashboard updates
    m_photonPoseEstimator = new PhotonPoseEstimator(DrivetrainConstants.TAG_LAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_photonCamera, DrivetrainConstants.ROBOT_TO_CAM);

    // Create a timer for less critical tasks such as Smartdashboard updates
    this.m_timer = new Timer();
    m_timer.start();
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
    //Publish pose for advantagescope odometry
    m_posePublisher.set(m_odometry.getEstimatedPosition());
    Logger.recordOutput("robotPose", m_odometry.getEstimatedPosition());
    
    // Ask Photon for a generated pose
    Optional<EstimatedRobotPose> estPose = m_photonPoseEstimator.update();

    // Checks if Photon returned a pose
    if (!estPose.isEmpty())
    {
      // Seeds an initial odometry value from vision system
      if (!m_odometrySeeded)
      {
        // Seed pose
        this.m_odometry.resetPosition(estPose.get().estimatedPose.getRotation().toRotation2d(), m_modulePositions,
            estPose.get().estimatedPose.toPose2d());
        m_odometrySeeded = true;
      } else
      {
        // Add vision to kalman filter
        addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), ModuleCount);


      }
    }
    // Report robots current pose to smartdashboard every half second
    if (m_timer.get() > 0.5)
    {
      m_timer.reset();
      SmartDashboard.putString("Pose - Vision", m_odometry.getEstimatedPosition().toString());
      SmartDashboard.putString("Pose - Drivetrain", m_odometry.getEstimatedPosition().toString());

      SmartDashboard.putBoolean("Od seeded", m_odometrySeeded);
    }
  }
}
