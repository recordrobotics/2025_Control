package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KillableSubsystem;

public class KillSpecified extends Command {

  private KillableSubsystem[] _subsytems;
  private Boolean _shouldContinuouslyExecute;

  /**
   * Kills all subsystems inputted
   *
   * @param subsystems
   */
  public KillSpecified(KillableSubsystem... subsystems) {
    this(false, subsystems);
  }

  /**
   * Kills all subsystems inputted
   *
   * @param execute whether or not the command should run continuously
   * @param subsystems
   */
  public KillSpecified(Boolean shouldContinuouslyExecute, KillableSubsystem... subsystems) {
    addRequirements(subsystems);
    _subsytems = subsystems;
    _shouldContinuouslyExecute = shouldContinuouslyExecute;
  }

  @Override
  public void initialize() {
    for (KillableSubsystem subsystem : _subsytems) {
      subsystem.kill();
    }
  }

  @Override
  public void execute() {
    for (KillableSubsystem subsystem : _subsytems) {
      subsystem.kill();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return !_shouldContinuouslyExecute;
  }
}
