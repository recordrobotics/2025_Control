package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.Constants;
import frc.robot.subsystems.Lights;

public class CoralShooterLights extends VirtualLightsSubsystem {

  public CoralShooterLights(Lights lights) {
    super(lights, 112, 117);
    setDefaultCommand(runPattern(Constants.Lights.ALLIANCE_COLOR).ignoringDisable(true));
  }
}
