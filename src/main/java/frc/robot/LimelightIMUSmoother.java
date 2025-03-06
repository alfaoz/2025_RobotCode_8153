// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers.IMUData;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * A class that smooths Limelight pose estimates using IMU data to reduce jitter.
 * This class detects when pose updates from the Limelight are too jittery and
 * uses IMU data to smooth them out.
 */
public class LimelightIMUSmoother {
    // NetworkTable entries for configuration
    private final NetworkTable m_configTable;
    private final BooleanEntry m_enabledEntry;
    private final DoubleEntry m_jitterThresholdEntry;
    private final DoubleEntry m_smoothingFactorEntry;
    private final StringEntry m_statusEntry;
    
    // Telemetry entries
    private final NetworkTable m_telemetryTable;
    private final DoubleEntry m_currentJitterEntry;
    private final DoubleEntry m_correctionMagnitudeEntry;
    private final BooleanEntry m_isCorrectingEntry;
    
    // State variables
    private Pose2d m_lastGoodPose = new Pose2d();
    private double m_lastGoodTimestamp = 0.0;
    private Rotation2d m_lastIMUYaw = new Rotation2d();
    private double m_lastIMUTimestamp = 0.0;
    private boolean m_hasValidPose = false;
    
    // Constants
    private static final double DEFAULT_JITTER_THRESHOLD = 0.05; // meters
    private static final double DEFAULT_SMOOTHING_FACTOR = 0.7; // 0.0 to 1.0
    private static final double MAX_TIME_BETWEEN_UPDATES = 1.0; // seconds
    
    /**
     * Creates a new LimelightIMUSmoother.
     * 
     * @param limelightName The name of the Limelight camera
     */
    public LimelightIMUSmoother(String limelightName) {
        // Set up NetworkTable entries for configuration
        m_configTable = NetworkTableInstance.getDefault().getTable("LimelightIMUSmoother");
        m_enabledEntry = m_configTable.getBooleanTopic("Enabled").getEntry(true);
        m_jitterThresholdEntry = m_configTable.getDoubleTopic("JitterThreshold").getEntry(DEFAULT_JITTER_THRESHOLD);
        m_smoothingFactorEntry = m_configTable.getDoubleTopic("SmoothingFactor").getEntry(DEFAULT_SMOOTHING_FACTOR);
        m_statusEntry = m_configTable.getStringTopic("Status").getEntry("Initializing");
        
        // Set up NetworkTable entries for telemetry
        m_telemetryTable = NetworkTableInstance.getDefault().getTable("LimelightIMUSmoother/Telemetry");
        m_currentJitterEntry = m_telemetryTable.getDoubleTopic("CurrentJitter").getEntry(0.0);
        m_correctionMagnitudeEntry = m_telemetryTable.getDoubleTopic("CorrectionMagnitude").getEntry(0.5);
        m_isCorrectingEntry = m_telemetryTable.getBooleanTopic("IsCorrectingJitter").getEntry(false);
        
        // Initialize IMU data
        updateIMUData(limelightName);
    }
    
    /**
     * Updates the IMU data from the Limelight.
     * 
     * @param limelightName The name of the Limelight camera
     */
    private void updateIMUData(String limelightName) {
        IMUData imuData = LimelightHelpers.getIMUData(limelightName);
        m_lastIMUYaw = Rotation2d.fromDegrees(imuData.Yaw);
        m_lastIMUTimestamp = Timer.getFPGATimestamp();
    }
    
    /**
     * Smooths a pose estimate from the Limelight using IMU data.
     * 
     * @param limelightName The name of the Limelight camera
     * @param poseEstimate The pose estimate from the Limelight
     * @return The smoothed pose estimate, or the original if smoothing is disabled
     */
    public PoseEstimate smoothPoseEstimate(String limelightName, PoseEstimate poseEstimate) {
        // Force a refresh of the enabled state from NetworkTables
        boolean enabled = m_enabledEntry.get(false);
        
        // If smoothing is disabled, return the original pose estimate
        if (!enabled) {
            m_statusEntry.set("Disabled");
            return poseEstimate;
        }
        
        // If we don't have a valid pose estimate, return the original
        if (poseEstimate == null) {
            m_statusEntry.set("No pose estimate");
            return null;
        }
        
        // Get the current IMU data
        IMUData imuData = LimelightHelpers.getIMUData(limelightName);
        Rotation2d currentIMUYaw = Rotation2d.fromDegrees(imuData.Yaw);
        double currentIMUTimestamp = Timer.getFPGATimestamp();
        
        // If this is the first valid pose, just store it and return
        if (!m_hasValidPose) {
            m_lastGoodPose = poseEstimate.pose;
            m_lastGoodTimestamp = poseEstimate.timestampSeconds;
            m_lastIMUYaw = currentIMUYaw;
            m_lastIMUTimestamp = currentIMUTimestamp;
            m_hasValidPose = true;
            m_statusEntry.set("First valid pose");
            return poseEstimate;
        }
        
        // If too much time has passed since the last update, reset
        if (poseEstimate.timestampSeconds - m_lastGoodTimestamp > MAX_TIME_BETWEEN_UPDATES) {
            m_lastGoodPose = poseEstimate.pose;
            m_lastGoodTimestamp = poseEstimate.timestampSeconds;
            m_lastIMUYaw = currentIMUYaw;
            m_lastIMUTimestamp = currentIMUTimestamp;
            m_statusEntry.set("Reset due to time gap");
            return poseEstimate;
        }
        
        // Calculate the expected pose based on IMU data
        Rotation2d imuYawDelta = currentIMUYaw.minus(m_lastIMUYaw);
        Pose2d expectedPose = new Pose2d(
            m_lastGoodPose.getTranslation(),
            m_lastGoodPose.getRotation().plus(imuYawDelta)
        );
        
        // Calculate the jitter (difference between expected and actual pose)
        double translationJitter = poseEstimate.pose.getTranslation().getDistance(expectedPose.getTranslation());
        double rotationJitter = Math.abs(poseEstimate.pose.getRotation().minus(expectedPose.getRotation()).getDegrees());
        
        // Update telemetry
        m_currentJitterEntry.set(translationJitter);
        
        // Get the jitter threshold from NetworkTables
        double jitterThreshold = m_jitterThresholdEntry.get();
        
        // If the jitter is below the threshold, update the last good pose and return the original
        if (translationJitter < jitterThreshold && rotationJitter < jitterThreshold * 10) {
            m_lastGoodPose = poseEstimate.pose;
            m_lastGoodTimestamp = poseEstimate.timestampSeconds;
            m_lastIMUYaw = currentIMUYaw;
            m_lastIMUTimestamp = currentIMUTimestamp;
            m_isCorrectingEntry.set(false);
            m_statusEntry.set("No jitter detected");
            return poseEstimate;
        }
        
        // If we're here, we have jitter and need to smooth it
        m_isCorrectingEntry.set(true);
        m_statusEntry.set("Smoothing jitter");
        
        // Get the smoothing factor from NetworkTables
        double smoothingFactor = m_smoothingFactorEntry.get();
        
        // Create a smoothed pose by blending the expected pose with the actual pose
        Translation2d smoothedTranslation = m_lastGoodPose.getTranslation().interpolate(
            poseEstimate.pose.getTranslation(), 1.0 - smoothingFactor);
        
        Rotation2d smoothedRotation = m_lastGoodPose.getRotation().interpolate(
            poseEstimate.pose.getRotation(), 1.0 - smoothingFactor);
        
        // Add the IMU rotation delta to the smoothed rotation
        smoothedRotation = smoothedRotation.plus(imuYawDelta);
        
        // Create the smoothed pose
        Pose2d smoothedPose = new Pose2d(smoothedTranslation, smoothedRotation);
        
        // Calculate the correction magnitude
        double correctionMagnitude = smoothedPose.getTranslation().getDistance(poseEstimate.pose.getTranslation());
        m_correctionMagnitudeEntry.set(correctionMagnitude);
        
        // Update the last good pose and IMU data
        m_lastGoodPose = smoothedPose;
        m_lastGoodTimestamp = poseEstimate.timestampSeconds;
        m_lastIMUYaw = currentIMUYaw;
        m_lastIMUTimestamp = currentIMUTimestamp;
        
        // Create a new pose estimate with the smoothed pose
        PoseEstimate smoothedPoseEstimate = new PoseEstimate(
            smoothedPose,
            poseEstimate.timestampSeconds,
            poseEstimate.latency,
            poseEstimate.tagCount,
            poseEstimate.tagSpan,
            poseEstimate.avgTagDist,
            poseEstimate.avgTagArea,
            poseEstimate.rawFiducials,
            poseEstimate.isMegaTag2
        );
        
        return smoothedPoseEstimate;
    }
    
    /**
     * Checks if the IMU smoother is enabled.
     * 
     * @return True if the IMU smoother is enabled, false otherwise
     */
    public boolean isEnabled() {
        return m_enabledEntry.get();
    }
    
    /**
     * Sets whether the IMU smoother is enabled.
     * 
     * @param enabled True to enable the IMU smoother, false to disable it
     */
    public void setEnabled(boolean enabled) {
        m_enabledEntry.set(enabled);
    }
    
    /**
     * Gets the current jitter threshold.
     * 
     * @return The current jitter threshold in meters
     */
    public double getJitterThreshold() {
        return m_jitterThresholdEntry.get();
    }
    
    /**
     * Sets the jitter threshold.
     * 
     * @param threshold The jitter threshold in meters
     */
    public void setJitterThreshold(double threshold) {
        m_jitterThresholdEntry.set(threshold);
    }
    
    /**
     * Gets the current smoothing factor.
     * 
     * @return The current smoothing factor (0.0 to 1.0)
     */
    public double getSmoothingFactor() {
        return m_smoothingFactorEntry.get();
    }
    
    /**
     * Sets the smoothing factor.
     * 
     * @param factor The smoothing factor (0.0 to 1.0)
     */
    public void setSmoothingFactor(double factor) {
        m_smoothingFactorEntry.set(Math.max(0.0, Math.min(1.0, factor)));
    }
    
    /**
     * Gets the current status of the IMU smoother.
     * 
     * @return The current status as a string
     */
    public String getStatus() {
        return m_statusEntry.get();
    }
}
