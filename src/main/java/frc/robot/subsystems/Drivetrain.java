// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Global;
import frc.robot.constants.drivetrain.DriveConstants;
import frc.robot.constants.drivetrain.TunerConstants;
import frc.robot.utils.Vision;

public class Drivetrain extends SwerveDrivetrain implements Subsystem 
{
  private final Vision m_Vision;

  private final Field2d m_Field = new Field2d();

  private boolean m_Aligned = false;
  private Pose2d m_CurrentSpeakerPose;

  private final Timer m_TelemetryTimer = new Timer();
  private final Timer m_ExtraTelemetryTimer = new Timer();
  private final Timer m_ApplicationTimer = new Timer();
  private final Timer m_VisionTimer = new Timer();

  private boolean hasAppliedPerspective = false;
  public int axisModifier = 1;

  private final SwerveRequest.ApplyChassisSpeeds m_AutoRequest = new SwerveRequest.ApplyChassisSpeeds()
    .withDriveRequestType(DriveRequestType.Velocity)
    .withSteerRequestType(SteerRequestType.MotionMagic);

  /** 
    @brief Creates a new Drivetrain.
    @param driveTrainConstants Drivetrain-wide constants for the swerve drive
    @param OdometryUpdateFrequency The frequency to run the odometry loop. If unspecified, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0
    @param modules Constants for each specific module 
  */
  public Drivetrain(Vision vision, SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules)
  {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    this.m_Vision = vision;

    m_ApplicationTimer.start();
    m_VisionTimer.start();

    if(Global.ENABLE_TELEMETRY)
    {
      m_TelemetryTimer.start();
    }
    if(Global.ENABLE_EXTRA_TELEMETRY)
    {
      m_ExtraTelemetryTimer.start();
    }

    configurePathPlanner();
  }

  @Override
  public void periodic()
  {
    // wait 5 seconds after code has initialized to begin using vision
    if(m_VisionTimer.hasElapsed(5))
    {
      updateOdometry();
    }

    if (!hasAppliedPerspective || DriverStation.isDisabled())
    {
      DriverStation.getAlliance().ifPresent
      (
        (allianceColor) -> {
          this.setOperatorPerspectiveForward
          (
            allianceColor == DriverStation.Alliance.Red ? 
            DriveConstants.RED_OPERATOR_FORWARD_PERSPECTIVE : DriveConstants.BLUE_OPERATOR_FORWARD_PERSPECTIVE
          );
          hasAppliedPerspective = true;
        }
      );
    }

    if(Global.ENABLE_TELEMETRY)
    {
      if(m_TelemetryTimer.get() > Global.TELEMETRY_UPDATE_SPEED)
      {
        m_TelemetryTimer.reset();
        m_Field.setRobotPose(m_odometry.getEstimatedPosition());
        Logger.recordOutput("robotPose", m_odometry.getEstimatedPosition());
      }
    }
    
    if(Global.ENABLE_EXTRA_TELEMETRY)
    {
      if(m_ExtraTelemetryTimer.get() > Global.EXTRA_TELEMETRY_UPDATE_SPEED)
      {
        m_ExtraTelemetryTimer.reset();
        m_Vision.logVision();
        SmartDashboard.putString("Applied Perspective", DriveConstants.PERSPECTIVE_MAP.get(this.m_operatorForwardDirection.getDegrees()));
        SmartDashboard.putNumber("Distance to Speaker", getDistanceToSpeakerMeters());
      }
    } 
  }

  // Vision
  private void updateOdometry()
  {
    EstimatedRobotPose pose;

    // empty vision cache (if it is empty, this should finish immediately)
    while ((pose = m_Vision.pollVision()) != null)
    {
      m_odometry.addVisionMeasurement
      (
          pose.estimatedPose.toPose2d(),
          pose.timestampSeconds
      );
    }
  }

  public Rotation2d getRotationToSpeaker()
  {
    return m_Vision.getRotationToPose(m_odometry.getEstimatedPosition(), m_CurrentSpeakerPose);
  }

  public double getDistanceToSpeakerMeters()
  {
    return m_Vision.getDistanceToPose(m_odometry.getEstimatedPosition(), m_CurrentSpeakerPose).baseUnitMagnitude();
  }
  
  public void setAligned(boolean aligned)
  {
    this.m_Aligned = aligned;
  }

  public boolean getAligned()
  {
    return this.m_Aligned;
  }

  public boolean getVisionEnabled()
  {
    return m_Vision.getVisionEnabled();
  }
  
  // Pathplanner
  public ChassisSpeeds getCurrentRobotChassisSpeeds()
  {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  private void configurePathPlanner()
  {
    //Determine the radius of the drivebase from module locations
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) 
    {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }
    
    // Create drivetrain object for pathplanner to use in its calculations
    AutoBuilder.configureHolonomic
    (
      () -> this.m_odometry.getEstimatedPosition(),
      this::seedFieldRelative,
      this::getCurrentRobotChassisSpeeds,
      (speeds) -> this.setControl(m_AutoRequest.withSpeeds(speeds)),
      new HolonomicPathFollowerConfig(new PIDConstants(7, 0, 0), new PIDConstants(7, 0, 0), TunerConstants.SPEED_AT_12_VOLTS_MPS, driveBaseRadius, new ReplanningConfig()),
      () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red,
      this
    );
  }
}
