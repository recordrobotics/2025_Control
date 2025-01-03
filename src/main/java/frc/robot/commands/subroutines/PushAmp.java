package frc.robot.commands.subroutines;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Crashbar;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import frc.robot.subsystems.Shooter.ShooterStates;

public class PushAmp extends SequentialCommandGroup {

  private static Channel _channel;
  private static Shooter _shooter;
  private static Crashbar _crashbar;

  /**
   * Command that assumes the flywheel and crashbar are already in position and moves the channel to shoot the note into the amp.
   * Then, turns everything off. 
   * @param channel
   * @param shooter
   * @param crashbar
   */
  public PushAmp (Channel channel, Shooter shooter, Crashbar crashbar, Double shootTime) {
    _channel = channel;
    _shooter = shooter;
    _crashbar = crashbar;

    final Runnable killSpecified = () -> new KillSpecified(_shooter, _channel, _crashbar);

    addCommands(
      new InstantCommand(()->_channel.toggle(ChannelStates.SHOOT), _channel).handleInterrupt(killSpecified),
      new WaitCommand(shootTime),
      new InstantCommand(()-> _shooter.toggle(ShooterStates.OFF), _shooter).handleInterrupt(killSpecified),
      new InstantCommand(()-> _channel.toggle(ChannelStates.OFF), _channel).handleInterrupt(killSpecified),
      new InstantCommand(()-> _crashbar.toggle(CrashbarStates.RETRACTED), _crashbar).handleInterrupt(killSpecified)
    );
  }

  public PushAmp (Channel channel, Shooter shooter, Crashbar crashbar) {
    this(channel, shooter, crashbar, 1.2);
  }
}