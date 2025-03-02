// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

/**
 * A command that turns on the Limelight lights when the trigger is pressed
 * and turns them off when the trigger is released (like a car headlight flasher).
 */
public class ToggleLimelightLightsCommand extends Command {
  private final String m_limelightName;

  /**
   * Creates a new ToggleLimelightLightsCommand.
   *
   * @param limelightName The name of the Limelight to control
   */
  public ToggleLimelightLightsCommand(String limelightName) {
    m_limelightName = limelightName;
  }

  // Called when the command is initially scheduled (trigger pressed).
  @Override
  public void initialize() {
    // Turn ON the lights when the trigger is pressed
    LimelightHelpers.setLEDMode_ForceOn(m_limelightName);
    SmartDashboard.putBoolean("Limelight/LightsOn", true);
    SmartDashboard.putString("Limelight/Name", m_limelightName);
  }

  // Called once the command ends or is interrupted (trigger released).
  @Override
  public void end(boolean interrupted) {
    // Turn OFF the lights when the trigger is released
    LimelightHelpers.setLEDMode_ForceOff(m_limelightName);
    SmartDashboard.putBoolean("Limelight/LightsOn", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command should run until the trigger is released
    return false;
  }
}
