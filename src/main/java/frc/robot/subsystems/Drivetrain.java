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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Global;
import frc.robot.constants.drivetrain.DriveConstants;
import frc.robot.constants.drivetrain.TunerConstants;

public class Drivetrain extends SwerveDrivetrain implements Subsystem 
{
  private PhotonCamera m_CameraForward1, m_CameraForward2, m_CameraRear1;
  private PhotonPoseEstimator m_CameraForward1Estimator, m_CameraForward2Estimator, m_CameraRear1Estimator;
<<<<<<< HEAD
  private final Field2d m_Field = new Field2d();

  private final Timer m_TelemetryTimer = new Timer();
=======
  private Timer m_TelemetryTimer = new Timer();
  Field2d m_Field_Odometry = new Field2d();
>>>>>>> 1b2a423d25c9bd655d4e996916bb9b2c0911e7de

  private final SwerveRequest.ApplyChassisSpeeds m_autoRequest = new SwerveRequest.ApplyChassisSpeeds();

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
  }

  @Override
  public void periodic()
  {
<<<<<<< HEAD
    this.updateVision();
=======
    // Check to see if CTRE swerve does this internally and calling it here would be redundant.
    // this.updateVision();
>>>>>>> 1b2a423d25c9bd655d4e996916bb9b2c0911e7de

    if(Global.ENABLE_TELEMETRY)
    {
      if(m_TelemetryTimer.get() > Global.TELEMETRY_UPDATE_SPEED)
      {
        Logger.recordOutput("robotPose", m_odometry.getEstimatedPosition());
<<<<<<< HEAD
        m_Field.setRobotPose(this.m_odometry.getEstimatedPosition());
        SmartDashboard.putData("Field Data", m_Field);
        m_TelemetryTimer.reset();
=======
        m_Field_Odometry.setRobotPose(this.m_odometry.getEstimatedPosition());
        SmartDashboard.putData("Field Data", m_Field_Odometry);
>>>>>>> 1b2a423d25c9bd655d4e996916bb9b2c0911e7de
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

  public SwerveDrivePoseEstimator getOdometry()
  {
    return m_odometry;
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds()
  {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  private void configurePhotonVision()
  {
    m_CameraForward1 = new PhotonCamera("CameraForward1");
    m_CameraForward2 = new PhotonCamera("CameraForward2");
    m_CameraRear1 = new PhotonCamera("CameraRear1");
    m_CameraForward1Estimator = new PhotonPoseEstimator(DriveConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_CameraForward1, DriveConstants.CAMERA_POSITIONS.FORWARD_1);
    m_CameraForward2Estimator = new PhotonPoseEstimator(DriveConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_CameraForward2, DriveConstants.CAMERA_POSITIONS.FORWARD_2);
    m_CameraRear1Estimator = new PhotonPoseEstimator(DriveConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_CameraRear1, DriveConstants.CAMERA_POSITIONS.REAR_1);
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
    
    // //Create drivetrain object for pathplanner to use in its calculations
    // AutoBuilder.configureHolonomic(
    //   ()->this.getState().Pose,
    //   this::seedFieldRelative,
    //   this::getCurrentRobotChassisSpeeds,
    //   (speeds)->this.setControl(m_autoRequest.withSpeeds(speeds)),
    //   new HolonomicPathFollowerConfig(new PIDConstants(7, 0, 0), new PIDConstants(7, 0, 0), TunerConstants.SPEED_AT_12_VOLTS_MPS, driveBaseRadius, new ReplanningConfig()),
    //   ()->false,
    //   this);

  }

  private void updateVision()
  {
    if(m_CameraForward1.isConnected())
    {
      Optional<EstimatedRobotPose> pose = this.m_CameraForward1Estimator.update();
      try
      {
        this.m_odometry.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), ModuleCount);
      }
      catch(Exception e)
      {
        // System.out.println("[DRIVE] WARNING: CameraForward1Estimator returning null values.");
      }
    }
    if(m_CameraForward2.isConnected())
    {
      Optional<EstimatedRobotPose> pose = this.m_CameraForward2Estimator.update();
      try
      {
        this.m_odometry.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), ModuleCount);
      }
      catch(Exception e)
      {
        // System.out.println("[DRIVE] WARNING: CameraForward2Estimator returning null values.");
      }
    }
    if(m_CameraRear1.isConnected())
    {
      Optional<EstimatedRobotPose> pose = this.m_CameraRear1Estimator.update();
      try
      {
        this.m_odometry.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), ModuleCount);
      }
      catch(Exception e)
      {
        // System.out.println("[DRIVE] WARNING: CameraRear1Estimator returning null values.");
      }
    }
  }
}
