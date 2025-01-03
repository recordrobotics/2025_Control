package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Crashbar.CrashbarStates;

public class ManualCrashbar extends Command {

  public ManualCrashbar() {
    addRequirements(RobotContainer.crashbar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.crashbar.toggle(CrashbarStates.EXTENDED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.crashbar.toggle(CrashbarStates.RETRACTED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
