// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A command that periodically updates the robot's position using Limelight data.
 * This improves the accuracy of the robot's position estimation, which is used by
 * PathPlanner for path following.
 */
public class LimelightPoseEstimatorCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final String m_limelightName;
  private final Telemetry m_telemetry;
  
  // Shuffleboard entries for displaying Limelight data
  private GenericEntry m_validPoseEntry;
  private GenericEntry m_poseXEntry;
  private GenericEntry m_poseYEntry;
  private GenericEntry m_poseRotEntry;
  private GenericEntry m_tagCountEntry;
  
  // Configuration parameters for filtering vision measurements
  private static final int MIN_TAG_COUNT = 1;        // Minimum number of tags required for a valid measurement
  private static final double MIN_TAG_SPAN = 0.1;    // Minimum span between tags in meters (reduced from 0.5)
  private static final double MAX_AMBIGUITY = 0.5;   // Maximum allowed ambiguity in tag detection (increased from 0.2)
  private static final double MAX_TAG_DISTANCE = 4.0; // Maximum distance to tags in meters (increased from 6.0)
  
  /**
   * Creates a new LimelightPoseEstimatorCommand.
   * 
   * @param drivetrain The drivetrain subsystem to update
   * @param limelightName The name of the Limelight camera
   */
  public LimelightPoseEstimatorCommand(CommandSwerveDrivetrain drivetrain, String limelightName) {
    this(drivetrain, limelightName, null);
  }
  
  /**
   * Creates a new LimelightPoseEstimatorCommand with telemetry support.
   * 
   * @param drivetrain The drivetrain subsystem to update
   * @param limelightName The name of the Limelight camera
   * @param telemetry The telemetry object to update with Limelight data
   */
  public LimelightPoseEstimatorCommand(CommandSwerveDrivetrain drivetrain, String limelightName, Telemetry telemetry) {
    m_drivetrain = drivetrain;
    m_limelightName = limelightName;
    m_telemetry = telemetry;
    
    // This command doesn't require any subsystems, as it only reads from the Limelight
    // and updates the drivetrain's pose estimator without interfering with other commands
  }

  /**
   * Sets the Shuffleboard entries to use for displaying Limelight data.
   * This should be called once from RobotContainer to set up the entries.
   * 
   * @param validPoseEntry Entry for displaying pose validity
   * @param poseXEntry Entry for displaying X position
   * @param poseYEntry Entry for displaying Y position
   * @param poseRotEntry Entry for displaying rotation
   * @param tagCountEntry Entry for displaying tag count
   */
  public void setShuffleboardEntries(
      GenericEntry validPoseEntry,
      GenericEntry poseXEntry,
      GenericEntry poseYEntry,
      GenericEntry poseRotEntry,
      GenericEntry tagCountEntry) {
    m_validPoseEntry = validPoseEntry;
    m_poseXEntry = poseXEntry;
    m_poseYEntry = poseYEntry;
    m_poseRotEntry = poseRotEntry;
    m_tagCountEntry = tagCountEntry;
  }

  @Override
  public void initialize() {
    // Update the SmartDashboard indicator for compatibility
    SmartDashboard.putBoolean("LIMELIGHT_VALID_POSE_INDICATOR", false);
  }

  @Override
  public void execute() {
    // Get the pose estimate from the Limelight
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);
    
    // Check if we have a valid pose estimate
    boolean isValid = (poseEstimate != null && isValidPoseEstimate(poseEstimate));
    
    // Update both SmartDashboard and Shuffleboard indicators
    SmartDashboard.putBoolean("LIMELIGHT_VALID_POSE", isValid);
    SmartDashboard.putBoolean("LIMELIGHT_VALID_POSE_INDICATOR", isValid);
    
    // Update the Shuffleboard indicator
    if (m_validPoseEntry != null) {
      m_validPoseEntry.setBoolean(isValid);
    }
    
    if (isValid) {
      // Get the pose and timestamp
      Pose2d pose = poseEstimate.pose;
      double timestamp = poseEstimate.timestampSeconds;
      
      // Add the vision measurement to the drivetrain's pose estimator
      m_drivetrain.addVisionMeasurement(pose, timestamp);
      
      // Update telemetry if available
      if (m_telemetry != null) {
        m_telemetry.updateLimelightPose(true, pose, poseEstimate.tagCount);
      }
      
      // Update Shuffleboard entries with pose data
      if (m_poseXEntry != null) {
        m_poseXEntry.setDouble(pose.getX());
      }
      
      if (m_poseYEntry != null) {
        m_poseYEntry.setDouble(pose.getY());
      }
      
      if (m_poseRotEntry != null) {
        m_poseRotEntry.setDouble(pose.getRotation().getDegrees());
      }
      
      if (m_tagCountEntry != null) {
        m_tagCountEntry.setDouble(poseEstimate.tagCount);
      }
      
      // Also update SmartDashboard for debugging
      SmartDashboard.putNumber("LIMELIGHT_POSE_X", pose.getX());
      SmartDashboard.putNumber("LIMELIGHT_POSE_Y", pose.getY());
      SmartDashboard.putNumber("LIMELIGHT_POSE_ROT", pose.getRotation().getDegrees());
      SmartDashboard.putNumber("LIMELIGHT_TAG_COUNT", poseEstimate.tagCount);
      
      // Print debug info to help troubleshoot
      // System.out.println("Limelight pose update: X=" + pose.getX() + ", Y=" + pose.getY() + ", Rot=" + pose.getRotation().getDegrees() + ", Tags=" + poseEstimate.tagCount);
    } else {
      // Update telemetry if available
      if (m_telemetry != null) {
        m_telemetry.updateLimelightPose(false, null, 0);
      }
      
      // Log that we don't have a valid pose
      if (poseEstimate == null) {
        // System.out.println("Limelight pose estimate is null");
      } else {
        // System.out.println("Limelight pose invalid: TagCount=" + poseEstimate.tagCount);
      }
    }
  }

  /**
   * Checks if a pose estimate is valid based on various quality metrics.
   * 
   * @param estimate The pose estimate to validate
   * @return True if the pose estimate is valid, false otherwise
   */
  private boolean isValidPoseEstimate(PoseEstimate estimate) {
    // Accept any pose estimate with at least one tag
    // This is a very lenient approach to ensure we get as many updates as possible
    if (estimate.tagCount < MIN_TAG_COUNT) {
      return false;
    }

    if (estimate.avgTagDist > MAX_TAG_DISTANCE){
      return false;
    }
    // Accept all poses regardless of distance
    // We'll let the pose estimator's built-in filtering handle any inaccuracies
    
    // All checks passed, the pose estimate is valid
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    // Nothing to clean up
  }

  @Override
  public boolean isFinished() {
    // This command never finishes on its own
    // It will run continuously until explicitly interrupted
    return false;
  }
  
  /**
   * Make this command run even when the robot is disabled.
   * This ensures continuous pose estimation at all times.
   */
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
