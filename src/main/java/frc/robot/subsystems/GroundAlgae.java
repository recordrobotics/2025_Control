package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.ShuffleboardPublisher;

public class GroundAlgae extends KillableSubsystem implements ShuffleboardPublisher {
  private Spark motor = new Spark(RobotMap.GroundAlgae.MOTOR_ID);
  private DigitalInput algaeDetector = new DigitalInput(RobotMap.GroundAlgae.LIMIT_SWITCH_ID);
  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.CoralIntake.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private static final double defaultSpeed = Constants.GroundAlgae.DEFAULT_SPEED;

  public GroundAlgae() {
    toggle(GroundAlgaeStates.OFF);
  }

  public enum GroundAlgaeStates {
    IN,
    OUT,
    OFF;
  }

  public void toggle(GroundAlgaeStates state, double speed) {
    switch (state) {
      case IN: // take in note
        motor.set(speed);
        break;
      case OUT: // push out note
        motor.set(-speed);
        break;
      case OFF: // turn off or kill
      default: // should never happen
        motor.set(0);
        break;
    }
  }

  public void toggle(GroundAlgaeStates state) {
    toggle(state, defaultSpeed);
  }

  public boolean hasAlgae() {
    return debounced_value;
  }

  public void periodic() {
    debounced_value = !m_debouncer.calculate(algaeDetector.get());
  }

  @Override
  public void kill() {
    toggle(GroundAlgaeStates.OFF);
  }

  /** frees up all hardware allocations */
  @Override
  public void close() {
    motor.close();
    algaeDetector.close();
  }

  @Override
  public void setupShuffleboard() {
    // TODO do we need other shuffleboard stuff
    DashboardUI.Test.addMotor("GroundAlgae", motor);
  }
}
