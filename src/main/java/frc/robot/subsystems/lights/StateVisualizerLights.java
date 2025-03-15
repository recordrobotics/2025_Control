package frc.robot.subsystems.lights;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;

public class StateVisualizerLights extends VirtualLightsSubsystem {

  public StateVisualizerLights(Lights lights) {
    super(lights, 0, 29);
    setDefaultCommand(runPattern(Constants.Lights.ALLIANCE_COLOR_FANCY).ignoringDisable(true));
  }
}
