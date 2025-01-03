package frc.robot.commands.notes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class AcquireSmart extends SequentialCommandGroup {

  private static Acquisition _acquisition;
  private static Channel _channel;
  private static Photosensor _photosensor;
  private static Shooter _shooter;

  /**
   * Command that acquires the note in a way that centers it within the channel.
   *
   * @param acquisition
   * @param channel
   * @param photosensor
   * @param shooter
   */
  public AcquireSmart(
      Acquisition acquisition, Channel channel, Photosensor photosensor, Shooter shooter) {
    _acquisition = acquisition;
    _channel = channel;
    _photosensor = photosensor;
    _shooter = shooter;

    final Runnable killSpecified = () -> new KillSpecified(_acquisition, _channel, _shooter);

    addCommands(
        // Reverse shooter to ensure note does not come out through the shooter
        new InstantCommand(() -> _shooter.toggle(ShooterStates.REVERSE), _shooter)
            .handleInterrupt(killSpecified),
        // Turns acq and channel on to make the note move into the robot
        new InstantCommand(() -> _acquisition.toggle(AcquisitionStates.IN), _acquisition)
            .handleInterrupt(killSpecified),
        new InstantCommand(() -> _channel.toggle(ChannelStates.SHOOT), _channel)
            .handleInterrupt(killSpecified),
        // Waits until photosensor
        new WaitUntilCommand(() -> _photosensor.getDebouncedValue()),
        // Turns acq off to prevent more notes from getting acquired
        new InstantCommand(() -> _acquisition.toggle(AcquisitionStates.OFF), _acquisition)
            .handleInterrupt(killSpecified),
        // Waits until photosensor off, and then extra 0.15 seconds
        // Wait until note moves fully into the shooter assembely, and then some
        new WaitUntilCommand(() -> !_photosensor.getDebouncedValue())
            .handleInterrupt(killSpecified),
        new WaitCommand(0.15),
        // Turns channel reverse to unsquish the note back to the middle of the channel
        new InstantCommand(() -> _channel.toggle(ChannelStates.REVERSE), _channel)
            .handleInterrupt(killSpecified),
        // Waits until photosensor on, then toggle channel off
        // This stops the note when it is centered in the channel
        new WaitUntilCommand(() -> _photosensor.getDebouncedValue()).handleInterrupt(killSpecified),
        new InstantCommand(() -> _channel.toggle(ChannelStates.OFF), _channel)
            .handleInterrupt(killSpecified),
        new InstantCommand(() -> _shooter.toggle(ShooterStates.OFF), _shooter)
            .handleInterrupt(killSpecified));
  }
}
// TODO: investigate what happens when interrupted
