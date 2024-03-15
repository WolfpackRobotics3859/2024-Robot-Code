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
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Global;
import frc.robot.constants.drivetrain.DriveConstants;
import frc.robot.constants.drivetrain.TunerConstants;

public class Drivetrain extends SwerveDrivetrain implements Subsystem 
{
  private PhotonCamera m_CameraRight1, m_CameraLeft1, m_CameraRear1;
  private PhotonPoseEstimator m_CameraRight1Estimator, m_CameraLeft1Estimator, m_CameraRear1Estimator;
  private Timer m_TelemetryTimer = new Timer();
  private Timer m_ExtraTelemetryTimer = new Timer();
  private int m_CameraRight1ExceptionCount, m_CameraLeft1ExceptionCount, m_CameraRear1ExceptionCount;
  private boolean m_VisionEnabled = true;
  private boolean m_Aligned = false;

  private final SwerveRequest.ApplyChassisSpeeds m_AutoRequest = new SwerveRequest.ApplyChassisSpeeds()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic);

  /** 
    @brief Creates a new Drivetrain.
    @param driveTrainConstants Drivetrain-wide constants for the swerve drive
    @param OdometryUpdateFrequency The frequency to run the odometry loop. If unspecified, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0
    @param modules Constants for each specific module 
  */
  public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules)
  {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    configurePhotonVision();
    configurePathPlanner();
    if(Global.ENABLE_TELEMETRY)
    {
      m_TelemetryTimer.start();
    }
    if(Global.ENABLE_EXTRA_TELEMETRY)
    {
      m_ExtraTelemetryTimer.start();
    }
  }

  @Override
  public void periodic()
  {
    m_CameraRight1ExceptionCount = updateVisionWithCamera(m_CameraRight1, m_CameraRight1Estimator, m_CameraRight1ExceptionCount);
    m_CameraLeft1ExceptionCount = updateVisionWithCamera(m_CameraLeft1, m_CameraLeft1Estimator, m_CameraLeft1ExceptionCount);
    m_CameraRight1ExceptionCount = updateVisionWithCamera(m_CameraRear1, m_CameraRear1Estimator, m_CameraRear1ExceptionCount);

    if(Global.ENABLE_TELEMETRY)
    {
      if(m_TelemetryTimer.get() > Global.TELEMETRY_UPDATE_SPEED)
      {
        Logger.recordOutput("robotPose", m_odometry.getEstimatedPosition());
      }
    }
    if(Global.ENABLE_EXTRA_TELEMETRY)
    {
      if(m_ExtraTelemetryTimer.get() > Global.EXTRA_TELEMETRY_UPDATE_SPEED)
      {
        SmartDashboard.putBoolean("CameraRight1 isConnected", m_CameraRight1.isConnected());
        SmartDashboard.putBoolean("CameraLeft1 isConnected", m_CameraLeft1.isConnected());
        SmartDashboard.putBoolean("CameraRear1 isConnected", m_CameraRear1.isConnected());
        SmartDashboard.putNumber("CameraRight1ExceptionCount", m_CameraRight1ExceptionCount);
        SmartDashboard.putNumber("CameraRight1ExceptionCount", m_CameraLeft1ExceptionCount);
        SmartDashboard.putNumber("CameraRight1ExceptionCount", m_CameraRear1ExceptionCount);
        SmartDashboard.putNumber("Yaw to speaker", this.yawToSpeaker.get().getDegrees());
        SmartDashboard.putNumber("Distance to Speaker", this.distanceToSpeaker.get());
        SmartDashboard.putBoolean("Vision Enabled", this.getVisionEnabled());
      }
    } 
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

  public Command applyAutoRequest()
  {
    return run(() -> this.setControl(m_AutoRequest));
  }

  public SwerveDrivePoseEstimator getOdometry()
  {
    return m_odometry;
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds()
  {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public Supplier<Rotation2d> yawToSpeaker = () -> this.m_odometry.getEstimatedPosition().getRotation().rotateBy(PhotonUtils.getYawToPose(this.m_odometry.getEstimatedPosition(), DriveConstants.APRIL_TAG_POSES.SPEAKER_POSE_SUPPLIER.get()));

  public Supplier<Double> distanceToSpeaker = () -> PhotonUtils.getDistanceToPose(this.m_odometry.getEstimatedPosition(), DriveConstants.APRIL_TAG_POSES.SPEAKER_POSE_SUPPLIER.get());
  
  public void setAligned(boolean aligned)
  {
    this.m_Aligned = aligned;
  }

  public boolean getAligned()
  {
    return this.m_Aligned;
  }
  
  private void configurePhotonVision()
  {
    m_CameraRight1 = new PhotonCamera("CameraRight1");
    m_CameraLeft1 = new PhotonCamera("CameraLeft1");
    m_CameraRear1 = new PhotonCamera("CameraRear1");
    m_CameraRight1Estimator = new PhotonPoseEstimator(DriveConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_CameraRight1, DriveConstants.CAMERA_POSITIONS.RIGHT_1);
    m_CameraLeft1Estimator = new PhotonPoseEstimator(DriveConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_CameraLeft1, DriveConstants.CAMERA_POSITIONS.LEFT_1);
    m_CameraRear1Estimator = new PhotonPoseEstimator(DriveConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_CameraRear1, DriveConstants.CAMERA_POSITIONS.REAR_1);
    m_CameraRight1ExceptionCount = m_CameraLeft1ExceptionCount = m_CameraRear1ExceptionCount = 0;
  }

  public boolean getVisionEnabled()
  {
    return this.m_VisionEnabled;
  }

  public void setVisionEnabled(boolean enabled)
  {
    this.m_VisionEnabled = enabled;
  }
  
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
    
    // Create drivetrain object for pathplanner to use in its calculations
    AutoBuilder.configureHolonomic(
      ()->this.m_odometry.getEstimatedPosition(),
      this::seedFieldRelative,
      this::getCurrentRobotChassisSpeeds,
      (speeds)->this.setControl(m_AutoRequest.withSpeeds(speeds)),
      new HolonomicPathFollowerConfig(new PIDConstants(7, 0, 0), new PIDConstants(7, 0, 0), TunerConstants.SPEED_AT_12_VOLTS_MPS, driveBaseRadius, new ReplanningConfig()),
      ()->true,
      this);
  }

  private int updateVisionWithCamera(PhotonCamera camera, PhotonPoseEstimator estimator, int exceptionCount)
  {
    Optional<EstimatedRobotPose> optionalPose;
    if(camera.isConnected())
    {
      optionalPose = estimator.update();
      try
      {
        if(optionalPose.isPresent())
        {
          EstimatedRobotPose pose = optionalPose.get();
          this.m_odometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        }
      }
      catch(Exception e)
      {
        if(exceptionCount == 0)
        {
          System.out.println("[DRIVE] WARNING: Something bad is happening with the " + camera.getName() + "estimator.");
        }
        return exceptionCount + 1;
      }
    }
    return exceptionCount;
  }
}
