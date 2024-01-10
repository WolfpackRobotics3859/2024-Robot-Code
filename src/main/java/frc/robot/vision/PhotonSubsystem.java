// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonSubsystem extends SubsystemBase {
  private PhotonPoseEstimator m_poseEstimator;
  private PhotonCamera m_photonCamera;
  //TODO move to constants
  private Transform3d robotToCam;
  private Pose3d currentPose;
  

  /** Creates a new PhotonSubsystem. */
  public PhotonSubsystem() {
    robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(8.5), 0.0, Units.inchesToMeters(33.5)), new Rotation3d(0, 15, 0)); //8.5 inches back from center, 33.5 inches up from center, facing forward and a 15 deg up
    m_photonCamera = new PhotonCamera("photonvision");
    //TODO get proper camera translation
    m_poseEstimator = new PhotonPoseEstimator(Constants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_photonCamera, robotToCam); //8.5 inches back from center, directly centered, 33in tall
    currentPose = new Pose3d(); 
  }

  public PhotonPipelineResult getLastestResult() 
  {
    return m_photonCamera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose() 
  {
    return m_poseEstimator.update();
  }

  public Pose3d getLastVisionMeasurement() {
    return currentPose;
  }
  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimatedPose = getEstimatedRobotPose();
    //If not empty, grab a pose result
    if(!estimatedPose.isEmpty()) {
      currentPose = estimatedPose.get().estimatedPose;

      SmartDashboard.putString("Pose", currentPose.toString());
    }
  }
}
