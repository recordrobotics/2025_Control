// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class StopAndWait extends Command {
  /** The timer used for waiting. */
  protected Timer m_timer = new Timer();
  private final double m_duration;
  private Drivetrain _drivetrain;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the
   * specified duration.
   * 
   * @param drivetrain the drivetrain object
   * @param seconds    the time to wait, in seconds
   */
  public StopAndWait(Drivetrain drivetrain, double seconds) {
    m_duration = seconds;
    _drivetrain = drivetrain;
  }

  @Override
  protected Object clone() {
    return new StopAndWait(_drivetrain, m_duration);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    _drivetrain.stop();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }

  @Override
  public boolean equals(Object other) {
    return other instanceof StopAndWait obj &&
        obj.m_duration == m_duration;
  }

  @Override
  public int hashCode() {
    return Double.valueOf(m_duration).hashCode();
  }
}