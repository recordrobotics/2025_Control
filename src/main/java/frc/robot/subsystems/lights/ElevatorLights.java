package frc.robot.subsystems.lights;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;

public class ElevatorLights extends VirtualLightsSubsystem {

  public ElevatorLights(Lights lights) {
    super(lights, 46, 93);
    setDefaultCommand(runPattern(Constants.Lights.ALLIANCE_COLOR_FANCY).ignoringDisable(true));
  }
}
