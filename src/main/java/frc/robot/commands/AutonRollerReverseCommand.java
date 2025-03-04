// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * An autonomous command that runs the roller in reverse mode for 1.5 seconds and then stops.
 * This command is designed specifically for autonomous routines.
 */
public class AutonRollerReverseCommand extends Command {
  private final SparkMax m_motor;
  private final Timer m_timer = new Timer();
  private static final double DURATION_SECONDS = 0.8;

  public AutonRollerReverseCommand(SparkMax motor) {
    m_motor = motor;
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_motor.set(0.35); // Same speed as CommandRollerReverse
  }

  @Override
  public void end(boolean interrupted) {
    m_motor.set(0);
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    // This command finishes after running for DURATION_SECONDS
    return m_timer.hasElapsed(DURATION_SECONDS);
  }
}
