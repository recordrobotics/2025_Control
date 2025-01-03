package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;

public class Channel extends KillableSubsystem implements ShuffleboardPublisher {
  private Spark channelMotor = new Spark(RobotMap.Channel.CHANNEL_MOTOR_ID);

  public Channel() {
    toggle(ChannelStates.OFF);
  }

  public enum ChannelStates {
    THROUGH,
    SHOOT,
    REVERSE,
    OFF;
  }

  public void toggle(double speed) {
    channelMotor.set(speed);
  }

  public void toggle(ChannelStates state) {
    switch (state) {
      case THROUGH: // move the note through
        toggle(Constants.Channel.THROUGH_SPEED);
        break;
      case SHOOT: // go fast to shoot the note
        toggle(Constants.Channel.SHOOT_SPEED);
        break;
      case REVERSE: // move the note away from the shooter
        toggle(Constants.Channel.REVERSE_SPEED);
        break;
      case OFF: // turn off or kill
      default: // should never happen
        toggle(0);
        break;
    }
  }

  @Override
  public void kill() {
    toggle(ChannelStates.OFF);
  }

  /** frees up all hardware allocations */
  public void close() {
    channelMotor.close();
  }

  @Override
  public void setupShuffleboard() {
    ShuffleboardUI.Test.addMotor("Channel", channelMotor);
  }
}
