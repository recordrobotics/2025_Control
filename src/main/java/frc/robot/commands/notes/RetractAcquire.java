package frc.robot.commands.notes;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Channel.ChannelStates;

public class RetractAcquire extends SequentialCommandGroup {

  private static Channel _channel;

  public RetractAcquire(Acquisition _acquisition, Channel channel) {
    _channel = channel;

    final Runnable killSpecified = () -> new KillSpecified(_channel);

    addCommands(
      // Reverse channel
      new InstantCommand(() -> _acquisition.toggle(AcquisitionStates.OFF), _acquisition).handleInterrupt(killSpecified),
      new InstantCommand(() -> _channel.toggle(ChannelStates.REVERSE), _channel).handleInterrupt(killSpecified),
      new WaitCommand(0.3),
      new InstantCommand(()->_channel.toggle(ChannelStates.OFF), _channel).handleInterrupt(killSpecified)
    );
  }
}