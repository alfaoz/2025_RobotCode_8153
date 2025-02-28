// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A command that aligns the robot's heading to be parallel with the plane of an AprilTag.
 * This makes the robot face directly towards the tag, which helps with alignment.
 * The driver can still control the robot's movement while the command handles the rotation.
 */
public class AlignWithAprilTagCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final String m_limelightName;
  private final PIDController m_rotationPID;
  private final double m_closeEnoughDistance; // Distance in meters to consider "close enough" to the tag
  private final FieldCentric m_drive; // Swerve request for field-centric control
  private final CommandXboxController m_joystick; // Controller for driver input
  private final double m_maxSpeed; // Maximum speed in meters per second

  // PID constants for rotation control
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.01;
  private static final double TOLERANCE = 1.0; // degrees

  /**
   * Creates a new AlignWithAprilTagCommand.
   * 
   * @param drivetrain The drivetrain subsystem to use
   * @param limelightName The name of the Limelight camera
   * @param closeEnoughDistance Distance in meters to consider "close enough" to the tag
   * @param drive The FieldCentric drive request to use
   * @param joystick The controller for driver input
   * @param maxSpeed The maximum speed in meters per second
   */
  public AlignWithAprilTagCommand(CommandSwerveDrivetrain drivetrain, String limelightName, 
                                 double closeEnoughDistance, FieldCentric drive,
                                 CommandXboxController joystick, double maxSpeed) {
    m_drivetrain = drivetrain;
    m_limelightName = limelightName;
    m_closeEnoughDistance = closeEnoughDistance;
    m_drive = drive;
    m_joystick = joystick;
    m_maxSpeed = maxSpeed;
    
    m_rotationPID = new PIDController(kP, kI, kD);
    m_rotationPID.setTolerance(TOLERANCE);
    m_rotationPID.enableContinuousInput(-180, 180); // Treat angle wrapping properly
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_rotationPID.reset();
  }

  @Override
  public void execute() {
    // Get driver input for movement
    double velocityX = -m_joystick.getLeftY() * m_maxSpeed;
    double velocityY = m_joystick.getLeftX() * m_maxSpeed;
    
    // Check if we have a valid target
    boolean hasValidTarget = LimelightHelpers.getTV(m_limelightName);
    SmartDashboard.putBoolean("AprilTag/HasTarget", hasValidTarget);
    
    if (!hasValidTarget) {
      // No valid target, allow manual rotation
      double rotationalRate = -m_joystick.getRightX() * m_maxSpeed;
      m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(rotationalRate));
      SmartDashboard.putBoolean("AprilTag/IsAligning", false);
      return;
    }

    // Get the target's pose in robot space
    double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(m_limelightName);
    
    if (targetPose.length < 6) {
      // Invalid pose data, allow manual rotation
      double rotationalRate = -m_joystick.getRightX() * m_maxSpeed;
      m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(rotationalRate));
      SmartDashboard.putBoolean("AprilTag/IsAligning", false);
      return;
    }

    // Calculate distance to target (using x and y components)
    double distance = Math.sqrt(targetPose[0] * targetPose[0] + targetPose[1] * targetPose[1]);
    SmartDashboard.putNumber("AprilTag/Distance", distance);
    
    // If we're close enough to the tag, we can stop aligning
    boolean isCloseEnough = distance < m_closeEnoughDistance;
    SmartDashboard.putBoolean("AprilTag/IsCloseEnough", isCloseEnough);
    
    if (isCloseEnough) {
      // Close enough, allow manual rotation
      double rotationalRate = -m_joystick.getRightX() * m_maxSpeed;
      m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(rotationalRate));
      SmartDashboard.putBoolean("AprilTag/IsAligning", false);
      return;
    }

    // Extract the yaw component from the target pose
    // The yaw represents the rotation around the Z axis
    double targetYaw = targetPose[5]; // In degrees
    
    // Calculate the heading error
    // We want to align with the tag, so our desired heading is 0 degrees relative to the tag
    // The error is the negative of the target's yaw in robot space
    double headingError = -targetYaw;
    SmartDashboard.putNumber("AprilTag/HeadingError", headingError);
    
    // Use PID to calculate the rotational rate
    double rotationRate = m_rotationPID.calculate(headingError, 0);
    SmartDashboard.putNumber("AprilTag/RotationRate", rotationRate);
    
    // Apply the rotation to the drivetrain while allowing driver control of movement
    m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(rotationRate));
    SmartDashboard.putBoolean("AprilTag/IsAligning", true);
  }

  @Override
  public void end(boolean interrupted) {
    // Let the default command take over when this command ends
    SmartDashboard.putBoolean("AprilTag/IsAligning", false);
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted
    return false;
  }
}
