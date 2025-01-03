package frc.robot.commands.notes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Shooter.ShooterStates;

public class AcquireSmart extends SequentialCommandGroup {

  /**
   * Command that acquires the note in a way that centers it within the channel.
   *
   * @param acquisition
   * @param channel
   * @param photosensor
   * @param shooter
   */
  public AcquireSmart() {
    final Runnable killSpecified =
        () ->
            new KillSpecified(
                RobotContainer.acquisition, RobotContainer.channel, RobotContainer.shooter);

    addCommands(
        // Reverse shooter to ensure note does not come out through the shooter
        new InstantCommand(
                () -> RobotContainer.shooter.toggle(ShooterStates.REVERSE), RobotContainer.shooter)
            .handleInterrupt(killSpecified),
        // Turns acq and channel on to make the note move into the robot
        new InstantCommand(
                () -> RobotContainer.acquisition.toggle(AcquisitionStates.IN),
                RobotContainer.acquisition)
            .handleInterrupt(killSpecified),
        new InstantCommand(
                () -> RobotContainer.channel.toggle(ChannelStates.SHOOT), RobotContainer.channel)
            .handleInterrupt(killSpecified),
        // Waits until photosensor
        new WaitUntilCommand(() -> RobotContainer.photosensor.getDebouncedValue()),
        // Turns acq off to prevent more notes from getting acquired
        new InstantCommand(
                () -> RobotContainer.acquisition.toggle(AcquisitionStates.OFF),
                RobotContainer.acquisition)
            .handleInterrupt(killSpecified),
        // Waits until photosensor off, and then extra 0.15 seconds
        // Wait until note moves fully into the shooter assembly, and then some
        new WaitUntilCommand(() -> !RobotContainer.photosensor.getDebouncedValue())
            .handleInterrupt(killSpecified),
        new WaitCommand(0.15),
        // Turns channel reverse to unsquish the note back to the middle of the channel
        new InstantCommand(
                () -> RobotContainer.channel.toggle(ChannelStates.REVERSE), RobotContainer.channel)
            .handleInterrupt(killSpecified),
        // Waits until photosensor on, then toggle channel off
        // This stops the note when it is centered in the channel
        new WaitUntilCommand(() -> RobotContainer.photosensor.getDebouncedValue())
            .handleInterrupt(killSpecified),
        new InstantCommand(
                () -> RobotContainer.channel.toggle(ChannelStates.OFF), RobotContainer.channel)
            .handleInterrupt(killSpecified),
        new InstantCommand(
                () -> RobotContainer.shooter.toggle(ShooterStates.OFF), RobotContainer.shooter)
            .handleInterrupt(killSpecified));
  }
}
// TODO: investigate what happens when interrupted
