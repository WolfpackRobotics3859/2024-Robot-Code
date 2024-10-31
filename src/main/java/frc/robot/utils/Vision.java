// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.vision.VisionConstants;

/**
 * A utility class for configuring, accessing, and performing all vision-related operations.
 */
public class Vision
{
    private final List<Camera> m_Cameras = new ArrayList<Camera>();
    
    private AprilTagFieldLayout m_FieldLayout;

    private ConcurrentLinkedQueue<EstimatedRobotPose> m_VisionCache = new ConcurrentLinkedQueue<>();

    private boolean visionEnabled = true;

    /**
     * Constructs a new Vision class.
     * @param camList A list containing pairs of the camera names and the Transform3d objects for each camera.
     */
    public Vision(ArrayList<Pair<String, Transform3d>> camList)
    {
        
        // load field layout
        try {
            m_FieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException err) {
            System.err.println("Failed to load field layout.");
            err.printStackTrace();
            return;
        }

        // create cameras
        camList.forEach(camera ->
            this.m_Cameras.add(new Camera(camera.getFirst(), camera.getSecond()))
        );

        // create seperate thread to run vision processing
        Thread m_Thread = new Thread
        (
            () -> {
                if (m_FieldLayout == null) return;
                while (!Thread.currentThread().isInterrupted())
                {
                    // update vision
                    updateVision();
                    try {
                        // sleep thread every 5 ms
                        Thread.sleep(VisionConstants.THREAD_SLEEP_TIME_MS);
                    } catch (InterruptedException err) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        );

        // set thread to daemon and lowest priority
        m_Thread.setDaemon(true);
        m_Thread.setPriority(VisionConstants.THREAD_PRIORITY);

        // start thread
        m_Thread.start();
    }

    /**
     * Grabs the head data in the vision cache, then deletes it from the cache
     * @return The oldest pose in the vision cache
     */
    public EstimatedRobotPose pollVision()
    {
        return m_VisionCache.poll();
    }

    /**
     * Adds new vision poses to the vision cache.
     */
    private void updateVision()
    {
        for (Camera camera : m_Cameras)
        {
            // if camera is disabled, ignore
            if (!camera.getEnabled()) continue;

            // get camera result
            PhotonPipelineResult result = camera.getResult();

            // if result has targets, process
            if(result.hasTargets())
            {
                // update pose estimator
                Optional<EstimatedRobotPose> optEst = camera.updateEstimator(result);
                if (optEst.isEmpty()) continue;

                EstimatedRobotPose est = optEst.get();

                // if result has only one target and the targets pose ambiguity is too high, continue
                if (est.targetsUsed.size() == 1
                && est.targetsUsed.get(0).getPoseAmbiguity() > VisionConstants.AMBIGUITY_THRESHOLD
                ) continue;
                
                // add estimation to vision cache
                m_VisionCache.add(est);
            }
        }
    }

    // CALCULATION FUNCTIONS

    /**
     * Calculates a field relative Rotation2d the robot should target to point at the given target pose.
     * @param robotPose The pose of the robot
     * @param targetPose The pose of the target
     * @return The Rotation2d object representing the rotation the robot needs to be facing relative to the field
     * to aim at the target
     */
    public Rotation2d getRotationToPose(Pose2d robotPose, Pose2d targetPose)
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
     * A class combining the PhotonCamera and PhotonPoseEstimator into one object.
     */
    public class Camera
    {
        private final String m_CameraName;

        private final PhotonCamera m_Camera;
        private final PhotonPoseEstimator m_PoseEstimator;

        private boolean enabled = true;

        /**
         * Constructs a new Camera object and configures the PhotonCamera and PhotonPoseEstimator.
         * @param cameraName The name of the camera in PhotonVision
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
         * Gets the latest result from the camera.
         * @return The latest result
         */
        public PhotonPipelineResult getResult()
        {
            return m_Camera.getLatestResult();
        }

        /**
         * Updates the PhotonPoseEstimator associated with the given camera.
         * @param result The result from the camera
         * @return The estimated robot pose (can be empty)
         */
        public Optional<EstimatedRobotPose> updateEstimator(PhotonPipelineResult result)
        {
            return m_PoseEstimator.update(result);
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