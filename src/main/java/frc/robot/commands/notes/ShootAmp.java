package frc.robot.commands.notes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.subroutines.PushAmp;
import frc.robot.commands.subroutines.SetupAmp;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Crashbar;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends SequentialCommandGroup {

  private static Channel _channel;
  private static Shooter _shooter;
  private static Crashbar _crashbar;

  /** Number of seconds it takes for the flywheel to spin up */
  private final double flywheelSpinupTime = 0.3;

  private final double crashbarExtendTime = 0.4;

  /** Number of seconds it takes to shoot once the flywheel h as been spun up */
  private final double shootTime = 0.7;

  /**
   * Command that shoots the note into the amp. Manages all relevant subsystems to do so (including
   * lowering crashbar, waiting for the shooter to speed up, etc)
   *
   * @param channel
   * @param shooter
   */
  public ShootAmp(Channel channel, Shooter shooter, Crashbar crashbar) {
    _channel = channel;
    _shooter = shooter;
    _crashbar = crashbar;
    addRequirements(channel);
    addRequirements(shooter);
    addRequirements(crashbar);

    addCommands(
        new SetupAmp(_shooter, _crashbar, true),
        new WaitCommand(Math.max(flywheelSpinupTime, crashbarExtendTime)),
        new PushAmp(_channel, _shooter, _crashbar, shootTime));
  }
}
