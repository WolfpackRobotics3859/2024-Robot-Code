// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {
  private PhotonPoseEstimator m_poseEstimator;
  private PhotonCamera m_photonCamera;
  /** Creates a new PhotonSubsystem. */
  public PhotonSubsystem() {
    
    Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(8.5), 0.0, Units.inchesToMeters(33.5)), new Rotation3d(0, 15, 0)); //8.5 inches back from center, 33.5 inches up from center, facing forward and a 15 deg up
    m_photonCamera = new PhotonCamera("photonvision");
    
    //TODO get camera translation
    try {
      m_poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_photonCamera, robotToCam); //8.5 inches back from center, directly centered, 33in tall
    } catch (IOException e) {
      e.printStackTrace();
    }
    
  }

  @Override
  public void periodic() {
    m_poseEstimator.update();
    // Throw some debugging data into smartdashboard before I implement poses into drive code with kalman
    try {
      SmartDashboard.putString("Pose", m_poseEstimator.update().get().estimatedPose.toString());
      SmartDashboard.putBoolean("hasTarget", m_photonCamera.getLatestResult().hasTargets());
      SmartDashboard.putNumber("numTargets", m_photonCamera.getLatestResult().targets.size());
      SmartDashboard.putNumber("confidenceBest", m_photonCamera.getLatestResult().getBestTarget().getPoseAmbiguity());
    } catch(Exception e) {

    }
    

    
  }
}
