package frc.robot.subsystems.lights;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;

public class CoralShooterLights extends VirtualLightsSubsystem {

  public CoralShooterLights(Lights lights) {
    super(lights, 112, 117);
    setDefaultCommand(runPattern(Constants.Lights.ALLIANCE_COLOR_FANCY).ignoringDisable(true));
  }
}
