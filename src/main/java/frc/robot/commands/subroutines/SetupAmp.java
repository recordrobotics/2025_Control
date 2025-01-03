package frc.robot.commands.subroutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Crashbar;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class SetupAmp extends SequentialCommandGroup {

  private static Shooter _shooter;
  private static Crashbar _crashbar;

  /**
   * Command that sets up to get ready for amp scoring by speeding up shooter and lowering crashbar
   * (if "lowerCrashbar" is set to true)
   *
   * @param shooter
   * @param crashbar
   * @param lowerCrashbar
   */
  public SetupAmp(Shooter shooter, Crashbar crashbar, Boolean lowerCrashbar) {
    _shooter = shooter;
    _crashbar = crashbar;

    final Runnable killSpecified = () -> new KillSpecified(_shooter, _crashbar);

    if (lowerCrashbar) {
      addCommands(
          new InstantCommand(
                  () -> {
                    shooter.toggle(ShooterStates.AMP);
                    crashbar.toggle(CrashbarStates.EXTENDED);
                  },
                  shooter,
                  crashbar)
              .handleInterrupt(killSpecified));
    } else {
      addCommands(
          new InstantCommand(
                  () -> {
                    shooter.toggle(ShooterStates.AMP);
                  },
                  shooter)
              .handleInterrupt(killSpecified));
    }
  }

  /**
   * Command that sets up to get ready for amp scoring by speeding up shooter and lowering crashbar
   *
   * @param shooter
   * @param crashbar
   * @param lowerCrashbar
   */
  public SetupAmp(Shooter shooter, Crashbar crashbar) {
    this(shooter, crashbar, true);
  }
}
