package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotMap;

public class PistonTemplate extends KillableSubsystem {

  // Sets up solenoid
  private DoubleSolenoid solenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          RobotMap.PistonTemplate.FORWARD_PORT,
          RobotMap.PistonTemplate.REVERSE_PORT);

  public PistonTemplate() {
    toggle(PistonTemplateStates.OFF);
  }

  public enum PistonTemplateStates {
    EXTENDED,
    RETRACTED,
    OFF;
  }

  public void toggle(PistonTemplateStates state) {
    switch (state) {
      case EXTENDED:
        solenoid.set(DoubleSolenoid.Value.kForward);
        break;
      case RETRACTED:
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
    toggle(PistonTemplateStates.OFF);
  }

  /** frees up all hardware allocations */
  public void close() {
    solenoid.close();
  }
}
