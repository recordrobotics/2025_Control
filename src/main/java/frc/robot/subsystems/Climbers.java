package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotMap;

public class Climbers extends KillableSubsystem {

  // Sets up solenoid
  private DoubleSolenoid solenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          RobotMap.Climbers.FORWARD_PORT,
          RobotMap.Climbers.REVERSE_PORT);

  public Climbers() {
    toggle(ClimberStates.OFF);
  }

  public enum ClimberStates {
    UP,
    DOWN,
    OFF;
  }

  public void toggle(ClimberStates state) {
    switch (state) {
      case UP:
        solenoid.set(DoubleSolenoid.Value.kForward);
        break;
      case DOWN:
        solenoid.set(DoubleSolenoid.Value.kReverse);
        break;
      case OFF: // turn off or kill
      default: // should never happen
        solenoid.set(DoubleSolenoid.Value.kOff);
        break;
    }
  }

  @Override
  public void kill() {
    toggle(ClimberStates.OFF);
  }

  /** frees up all hardware allocations */
  public void close() {
    solenoid.close();
  }
}
