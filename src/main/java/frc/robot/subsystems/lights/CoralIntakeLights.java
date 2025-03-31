package frc.robot.subsystems.lights;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;

public class CoralIntakeLights extends VirtualLightsSubsystem {

  public CoralIntakeLights(Lights lights) {
    super(lights, 30, 45);
    setDefaultCommand(
        runPattern(Constants.Lights.ALLIANCE_COLOR_FANCY_WITH_CLIMB).ignoringDisable(true));
  }
}
