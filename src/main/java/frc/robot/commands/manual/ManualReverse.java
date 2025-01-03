package frc.robot.commands.manual;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Channel.ChannelStates;

public class ManualReverse extends Command {

  private static Acquisition _acquisition;
  private static Channel _channel;

  public ManualReverse (Acquisition acquisition, Channel channel) {
    _acquisition = acquisition;
    _channel = channel;
    addRequirements(acquisition);
    addRequirements(channel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _acquisition.toggle(AcquisitionStates.REVERSE);
    _channel.toggle(ChannelStates.REVERSE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _acquisition.toggle(AcquisitionStates.OFF);
    _channel.toggle(ChannelStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}