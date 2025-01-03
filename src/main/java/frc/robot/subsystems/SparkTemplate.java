package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;

public class SparkTemplate extends KillableSubsystem implements ShuffleboardPublisher {
  private Spark motor = new Spark(RobotMap.SparkTemplate.MOTOR_ID);
  private static final double defaultSpeed = Constants.SparkTemplate.SPEED;
  private SparkTemplateStates currentState = SparkTemplateStates.OFF;

  public SparkTemplateStates getSparkTemplateState() {
    return currentState;
  }

  public SparkTemplate() {
    toggle(SparkTemplateStates.OFF);
  }

  public enum SparkTemplateStates {
    FORWARD,
    REVERSE,
    OFF;
  }

  public void toggle(SparkTemplateStates state, double speed) {
    currentState = state;
    switch (state) {
      case FORWARD:
        motor.set(speed);
        break;
      case REVERSE:
        motor.set(-speed);
        break;
      case OFF: // turn off or kill
      default: // should never happen
        motor.set(0);
        break;
    }
  }

  public void toggle(SparkTemplateStates state) {
    toggle(state, defaultSpeed);
  }

  @Override
  public void kill() {
    toggle(SparkTemplateStates.OFF);
  }

  /** frees up all hardware allocations */
  @Override
  public void close() {
    motor.close();
  }

  @Override
  public void setupShuffleboard() {
    ShuffleboardUI.Overview.setSparkTemplate(() -> motor.get() != 0);
    ShuffleboardUI.Autonomous.setSparkTemplate(() -> motor.get() != 0);
    ShuffleboardUI.Test.addMotor("SparkTemplate", motor);
  }
}
