package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionPanel extends SubsystemBase {
  private static final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

  /** frees up all hardware allocations */
  public void close() {
    pdp.close();
  }
}
