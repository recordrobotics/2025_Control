package frc.robot.commands.notes;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel.ChannelStates;

public class PushAcquire extends SequentialCommandGroup {

  private static Acquisition _acquisition;
  private static Channel _channel;

  /**
   * Command that picks up the note, steopping the acquision once note is acquired
   * @param acquisition
   * @param channel
   * @param photosensor
   */
  public PushAcquire (Acquisition acquisition, Channel channel) {
    _acquisition = acquisition;
    _channel = channel;

    final Runnable killSpecified = () -> new KillSpecified(_acquisition, _channel);

    addCommands(
      // Turns acq on
      new InstantCommand(() -> _acquisition.toggle(AcquisitionStates.IN), _acquisition).handleInterrupt(killSpecified),
      new InstantCommand(() -> _channel.toggle(ChannelStates.SHOOT), _channel).handleInterrupt(killSpecified)
    );
  }
}