package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Killable;

public class KillSpecified extends Command {

  private Killable[] _subsytems;
  private Boolean _shouldContinuouslyExecute;

  /**
   * Kills all subsystems inputted
   *
   * @param subsystems
   */
  public KillSpecified(Killable... subsystems) {
    this(false, subsystems);
  }

  /**
   * Kills all subsystems inputted
   *
   * @param execute whether or not the command should run continuously
   * @param subsystems
   */
  public KillSpecified(Boolean shouldContinuouslyExecute, Killable... subsystems) {
    
    try {
      addRequirements((SubsystemBase[]) subsystems);
    } catch (ClassCastException e) {
      // TODO: idk how logs
      System.out.println("Only use this for subsystems");
    }

    _subsytems = subsystems;
    _shouldContinuouslyExecute = shouldContinuouslyExecute;
  }

  @Override
  public void initialize() {
    for (Killable subsystem : _subsytems) {
      subsystem.kill();
    }
  }

  @Override
  public void execute() {
    for (Killable subsystem : _subsytems) {
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
