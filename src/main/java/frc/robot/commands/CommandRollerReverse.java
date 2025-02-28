// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that aligns the robot's heading to be parallel with the plane of an AprilTag.
 * This makes the robot face directly towards the tag, which helps with alignment.
 * The driver can still control the robot's movement while the command handles the rotation.
 */
public class CommandRollerReverse extends Command {
  private final SparkMax m_motor;
  public CommandRollerReverse(SparkMax motor){
    m_motor = motor;
  }

  @Override
  public void initialize() {
    m_motor.set(0.35);
  }

  @Override
  public void end(boolean interrupted) {
    m_motor.set(0);
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted
    return false;
  }
}
