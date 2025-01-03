package frc.robot.commands.notes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Photosensor;

public class Acquire extends SequentialCommandGroup {

  private static Acquisition _acquisition;
  private static Channel _channel;
  private static Photosensor _photosensor;

  /**
   * Command that picks up the note, steopping the acquision once note is acquired
   *
   * @param acquisition
   * @param channel
   * @param photosensor
   */
  public Acquire(Acquisition acquisition, Channel channel, Photosensor photosensor) {
    _acquisition = acquisition;
    _channel = channel;
    _photosensor = photosensor;

    final Runnable killSpecified = () -> new KillSpecified(_acquisition, _channel);

    addCommands(
        // Turns acq and channel on
        new InstantCommand(() -> _acquisition.toggle(AcquisitionStates.IN), _acquisition)
            .handleInterrupt(killSpecified),
        new InstantCommand(() -> _channel.toggle(ChannelStates.SHOOT), _channel)
            .handleInterrupt(killSpecified),
        // Waits until photosensor
        new WaitUntilCommand(() -> !_photosensor.getCurrentValue()),
        // Turns acq and channel off
        new InstantCommand(() -> _acquisition.toggle(AcquisitionStates.OFF), _acquisition)
            .handleInterrupt(killSpecified),
        new InstantCommand(() -> _channel.toggle(ChannelStates.OFF), _channel)
            .handleInterrupt(killSpecified));
  }
}
