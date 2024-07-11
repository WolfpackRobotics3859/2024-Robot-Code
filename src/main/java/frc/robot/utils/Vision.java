// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.vision.VisionConstants;

/**
 * A utility class for configuring, accessing, and performing all vision-related operations.
 */
public class Vision
{
    private final List<Camera> m_Cameras = new ArrayList<Camera>();
    
    private boolean visionEnabled = true;

    /**
     * Constructs a new Vision class.
     * @param camList A list containing pairs of the camera names and the Transform3d objects for each camera.
     */
    public Vision(ArrayList<Pair<String, Transform3d>> camList)
    {
        camList.forEach(camera ->
            this.m_Cameras.add(new Camera(camera.getFirst(), camera.getSecond()))
        );
    }

    /**
     * Gets the estimated poses from each camera.
     * @return The list of the estimated poses each camera is returning. May be empty.
     */
    private ArrayList<EstimatedRobotPose> getEstimatedPoses()
    {
        ArrayList<EstimatedRobotPose> estimatedPoses = new ArrayList<>();

        for (int i = 0; i < m_Cameras.size(); i++)
        {
            // get the poses from the camera
            Optional<EstimatedRobotPose> estimatedPose = m_Cameras.get(i).getEstimatedPose();

            // if poses are present and not null
            if (estimatedPose.isPresent() && estimatedPose.get().estimatedPose != null)
            {
                // add pose to list
                estimatedPoses.add(estimatedPose.get());
            }
        }

        return estimatedPoses;
    }

    /**
     * Adds vision measurements to the given SwerveDrivePoseEstimator.
     * @param poseEstimator The pose estimator to update
     */
    public void updateOdometry(SwerveDrivePoseEstimator poseEstimator)
    {
        ArrayList<EstimatedRobotPose> estimatedPoses = getEstimatedPoses();

        // return if the estimated pose list is empty or has more poses than cameras
        if(estimatedPoses.isEmpty() || estimatedPoses.size() > m_Cameras.size())
        {
            return;
        }

        for (int i = 0; i < estimatedPoses.size(); i++)
        {
            EstimatedRobotPose estimatedPose = estimatedPoses.get(i);
            
            // if pose doesn't exist continue
            if(estimatedPose.timestampSeconds < 0 || Timer.getFPGATimestamp() < estimatedPose.timestampSeconds || Timer.getFPGATimestamp() > estimatedPose.timestampSeconds + 1)
            {
                continue;
            }

            poseEstimator.addVisionMeasurement
            (
                estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds
            );
        }
    }

    /**
     * Calculates a field relative Rotation2d the robot should target to point at the given target pose.
     * @param robotPose The pose of the robot
     * @param targetPose The pose of the target
     * @return The Rotation2d object representing the rotation the robot needs to be facing relative to the field
     * to aim at the target
     */
    public Rotation2d getRotationToTarget(Pose2d robotPose, Pose2d targetPose)
    {
        // get differences between translations
        double dx = targetPose.getTranslation().getX() - robotPose.getTranslation().getX();
        double dy = targetPose.getTranslation().getY() - robotPose.getTranslation().getY();

        // find the robot relative angle based on the translational differences
        double robotRelativeAngle = Math.atan2(dy, dx);

        // subtract current robot angle from calculated angle to get field relative angle
        double targetAngle = robotRelativeAngle - robotPose.getRotation().getRadians();

        return Rotation2d.fromRadians(targetAngle);
    }

    /**
     * Gets the distance between the robot and a given target pose
     * @param robotPose The pose of the robot
     * @param targetPose The pose of the target
     * @return The distance measure between the robot and the target
     */
    public Measure<Distance> getDistanceToPose(Pose2d robotPose, Pose2d targetPose)
    {
        return Units.Meters.of(robotPose.getTranslation().getDistance(targetPose.getTranslation()));
    }

    /**
     * Sets the enabled state of the entire vision system
     * @param enable The enable state to set the vision system to
     */
    public void setVisionEnabled(boolean enable)
    {
        this.visionEnabled = enable;
    }

    /**
     * Gives the enabled state of the entire vision system
     * @return The enabled state
     */
    public boolean getVisionEnabled()
    {
        return this.visionEnabled;
    }

    /**
     * Sets the enabled state of an indivual camera at the given index
     * @param index The index of the camera in the camera list
     * @param enable The enable state to set the camera to
     */
    public void setCamEnabled(int index, boolean enable)
    {
        try
        {
            this.m_Cameras.get(index).setEnabled(enable);
        }
        catch (IndexOutOfBoundsException e)
        {
            DriverStation.reportWarning("[VISION] Camera index " + index + " is out of bounds", false);
        }
    }

    /**
     * Gets the enabled state of the camera at the given index.
     * @param index The index of the camera in the camera list
     * @return The enable state of the camera
     */
    public boolean getCamEnabled(int index)
    {
        try
        {
            return this.m_Cameras.get(index).getEnabled();
        }
        catch (IndexOutOfBoundsException e)
        {
            DriverStation.reportWarning("[VISION] Camera index " + index + " is out of bounds", false);
            return false;
        }
    }

    /**
     * Logs all important vision data to SmartDashboard.
     */
    public void logVision()
    {
        m_Cameras.forEach(camera ->
            SmartDashboard.putBoolean(camera.getName() + " enabled", camera.getEnabled())
        );

        SmartDashboard.putBoolean("Vision Enabled", this.visionEnabled);
    }

    /**
     * A class combining the PhotonCamera and PhotonPoseEstimator into one object, as well as providing
     * simple functionality for getting estimated robot position from each camera and enabling/disabling
     * individual cameras.
     */
    public class Camera
    {
        private final String m_CameraName;

        private final PhotonCamera m_Camera;
        private final PhotonPoseEstimator m_PoseEstimator;

        private boolean enabled = true;

        /**
         * Constructs a new Camera object and configures the PhotonCamera and PhotonPoseEstimator.
         * @param cameraName The name of the camera in PhotonVision.
         * @param robotToCam The Transform3d of the offset of the camera relative to the robot
         */
        public Camera(String cameraName, Transform3d robotToCam)
        {
            this.m_CameraName = cameraName;

            this.m_Camera = new PhotonCamera(cameraName);

            this.m_PoseEstimator = new PhotonPoseEstimator
            (
                VisionConstants.FIELD_LAYOUT,
                VisionConstants.POSE_STRATEGY,
                m_Camera,
                robotToCam
            );

            this.m_PoseEstimator.setMultiTagFallbackStrategy(VisionConstants.FALLBACK_POSE_STRATEGY);
        }

        /**
         * Gets the estimated pose from the PhotonPoseEstimator. This function also trims all
         * visible targets with pose ambiguity higher than the set threshold. Returns empty if:
         * <ul>
         *   <li>The timestamp of the provided pipeline result is the same as in the previous call to
         *       {@code update()}.
         *   <li>No targets are visible by the camera.
         *   <li>All targets visible by the camera have pose ambiguities greater than the threshold.
         * </ul>
         * @return The estimated pose
         */
        public Optional<EstimatedRobotPose> getEstimatedPose()
        {
            // if camera is disabled return empty
            if (!enabled)
            {
                return Optional.empty();
            }

            PhotonPipelineResult result = m_Camera.getLatestResult();

            // if no targets or timestap is less than 0 return empty
            if (!result.hasTargets() || result.getTimestampSeconds() < 0)
            {
                return Optional.empty();
            }

            // trim all targets from the result that have an ambiguity higher than 0.2
            result.targets.removeIf(target -> target.getPoseAmbiguity() > VisionConstants.AMBIGUITY_THRESHOLD);

            // if trimmed list has no targets return empty
            if(result.targets.size() == 0)
            {
                return Optional.empty();
            }
            
            return this.m_PoseEstimator.update(result);
        }

        /**
         * Sets the enable state of the camera. Useful if only one camera is bad and needs to be disabled.
         * @param enable The desired enable state
         */
        public void setEnabled(boolean enable)
        {
            this.enabled = enable;
        }

        /**
         * Gets the enabled state of the camera.
         * @return The enabled state of the camera
         */
        public boolean getEnabled()
        {
            return this.enabled;
        }

        /**
         * Gets the name of the camera.
         * @return The name of the camera
         */
        public String getName()
        {
            return this.m_CameraName;
        }
    }
}