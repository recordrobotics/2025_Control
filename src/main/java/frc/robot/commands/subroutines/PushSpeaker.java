package frc.robot.commands.subroutines;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Shooter.ShooterStates;

public class PushSpeaker extends SequentialCommandGroup {

  private static Channel _channel;
  private static Shooter _shooter;

  /**
   * Command that assumes the flywheel is already on and moves the channel to shoot the note into the speaker. 
   * Then, turns everything off.
   * @param channel
   * @param shooter
   */
  public PushSpeaker (Channel channel, Shooter shooter, Double shootTime) {
    _channel = channel;
    _shooter = shooter;

    final Runnable killSpecified = () -> new KillSpecified(_shooter, _channel);

    addCommands(
      new InstantCommand(()->_channel.toggle(ChannelStates.SHOOT), _channel).handleInterrupt(killSpecified),
      new WaitCommand(shootTime),
      new InstantCommand(()-> _shooter.toggle(ShooterStates.OFF), _shooter).handleInterrupt(killSpecified),
      new InstantCommand(()-> _channel.toggle(ChannelStates.OFF), _channel).handleInterrupt(killSpecified)
    );
  }

  public PushSpeaker (Channel channel, Shooter shooter) {
    this(channel, shooter, 1.0);
  }
}