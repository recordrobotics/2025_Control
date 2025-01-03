package frc.robot.commands.notes;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel.ChannelStates;

public class AcquireSmart extends SequentialCommandGroup {

  private static Acquisition _acquisition;
  private static Channel _channel;
  private static Photosensor _photosensor;
  private static Shooter _shooter;

  /**
   * Command that acquires the note in a way that centers it within the channel. 
   * @param acquisition
   * @param channel
   * @param photosensor
   * @param shooter
   */
  public AcquireSmart (Acquisition acquisition, Channel channel, Photosensor photosensor, Shooter shooter) {
    _acquisition = acquisition;
    _channel = channel;
    _photosensor = photosensor;
    _shooter = shooter;

    final Runnable killSpecified = () -> new KillSpecified(_acquisition, _channel, _shooter);

    addCommands(
      // Turns acq on
      new InstantCommand(()-> _shooter.toggle(ShooterStates.REVERSE), _shooter).handleInterrupt(killSpecified),
      new InstantCommand(() -> _acquisition.toggle(AcquisitionStates.IN), _acquisition).handleInterrupt(killSpecified),
      new InstantCommand(() -> _channel.toggle(ChannelStates.SHOOT), _channel).handleInterrupt(killSpecified),
      // Waits until photosensor
      new WaitUntilCommand(()->_photosensor.getDebouncedValue()),
      // Turns acq off
      new InstantCommand(() -> _acquisition.toggle(AcquisitionStates.OFF), _acquisition).handleInterrupt(killSpecified),
      // waits until photosensor off, extra time
      new WaitUntilCommand(()->!_photosensor.getDebouncedValue()).handleInterrupt(killSpecified),
      new WaitCommand(0.15),
      // Turns channel reverse
      new InstantCommand(() -> _channel.toggle(ChannelStates.REVERSE), _channel).handleInterrupt(killSpecified),
      // Waits until photosensor on, then toggle channel off
      new WaitUntilCommand(()->_photosensor.getDebouncedValue()).handleInterrupt(killSpecified),
      new InstantCommand(()-> _channel.toggle(ChannelStates.OFF), _channel).handleInterrupt(killSpecified),
      new InstantCommand(()-> _shooter.toggle(ShooterStates.OFF), _shooter).handleInterrupt(killSpecified)
    );
  }
}