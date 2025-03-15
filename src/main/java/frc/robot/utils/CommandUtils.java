package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class CommandUtils {

  /**
   * Wraps a command to ensure that it finishes when interrupted. This is useful for commands that
   * may not finish on their own when interrupted, such as commands that wait for a condition or
   * timeout
   *
   * @param command the command to wrap
   * @return a new WrapperCommand that finishes when interrupted
   */
  public static WrapperCommand finishOnInterrupt(Command command) {
    return new WrapperCommand(command) {
      private boolean finished = false;

      @Override
      public boolean isFinished() {
        return super.isFinished() || finished;
      }

      @Override
      public void end(boolean interrupted) {
        finished = true;
        super.end(interrupted);
      }
    };
  }
}
