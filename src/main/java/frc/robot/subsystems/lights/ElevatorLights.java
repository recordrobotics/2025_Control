package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.Lights;

public class ElevatorLights extends VirtualLightsSubsystem {

  public ElevatorLights(Lights lights) {
    super(lights, 46, 93);
    setDefaultCommand(runPattern(LEDPattern.kOff).ignoringDisable(true));
  }
}
