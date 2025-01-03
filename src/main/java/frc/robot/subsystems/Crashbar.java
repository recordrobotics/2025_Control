package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotMap;

public class Crashbar extends KillableSubsystem {

  // Creates solenoid
  private DoubleSolenoid solenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          RobotMap.Crashbar.FORWARD_PORT,
          RobotMap.Crashbar.REVERSE_PORT);

  public Crashbar() {
    toggle(CrashbarStates.OFF);
  }

  public enum CrashbarStates {
    EXTENDED,
    RETRACTED,
    OFF;
  }

  public void toggle(CrashbarStates state) {
    switch (state) {
      case EXTENDED: // prepare for amp shot
        solenoid.set(DoubleSolenoid.Value.kForward);
        break;
      case RETRACTED: // move crashbar out of the way
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
    toggle(CrashbarStates.OFF);
  }

  /** frees up all hardware allocations */
  public void close() {
    solenoid.close();
  }
}
