package frc.robot.subsystems.lights;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;

public class AlgaeGrabberLights extends VirtualLightsSubsystem {

  public AlgaeGrabberLights(Lights lights) {
    super(lights, 94, 111);
    setDefaultCommand(runPattern(Constants.Lights.ALLIANCE_COLOR).ignoringDisable(true));
  }
}
