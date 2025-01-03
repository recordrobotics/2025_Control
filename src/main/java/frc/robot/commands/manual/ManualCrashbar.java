package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Crashbar;
import frc.robot.subsystems.Crashbar.CrashbarStates;

public class ManualCrashbar extends Command {

  private Crashbar _crashbar;

  public ManualCrashbar(Crashbar crashbar) {
    _crashbar = crashbar;
    addRequirements(crashbar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _crashbar.toggle(CrashbarStates.EXTENDED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _crashbar.toggle(CrashbarStates.RETRACTED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
