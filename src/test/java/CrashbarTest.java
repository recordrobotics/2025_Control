import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.RobotMap;
import frc.robot.subsystems.Crashbar;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class CrashbarTest {
  private Crashbar crashbar;
  private DoubleSolenoidSim pistonSim;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    crashbar = new Crashbar();
    pistonSim =
        new DoubleSolenoidSim(
            PneumaticsModuleType.CTREPCM,
            RobotMap.Crashbar.FORWARD_PORT,
            RobotMap.Crashbar.REVERSE_PORT); // Simulate the piston with DoubleSolenoidSim
  }

  @AfterEach
  public void shutdown() throws Exception {
    crashbar.close();
  }

  private void check(Value correct, String msg) {
    assertEquals(correct, pistonSim.get(), msg);
  }

  @Test
  public void testInitialState() {
    check(Value.kOff, "Motor should be off initially.");
  }

  @Test
  public void testToggleExtendedState() {
    crashbar.toggle(CrashbarStates.EXTENDED);
    check(Value.kForward, "Motor should be set to EXTENDED.");
  }

  @Test
  public void testToggleRetractedState() {
    crashbar.toggle(CrashbarStates.RETRACTED);
    check(Value.kReverse, "Motor should be set to RETRACTED.");
  }

  @Test
  public void testToggleOffState() {
    crashbar.toggle(CrashbarStates.EXTENDED); // Set to EXTENDED first
    crashbar.toggle(CrashbarStates.OFF); // Now set to OFF
    check(Value.kOff, "Motor should be off when state is OFF.");
  }

  @Test
  public void testKill() {
    crashbar.toggle(CrashbarStates.EXTENDED); // Set to EXTENDED first
    crashbar.kill(); // Kill should turn it off
    check(Value.kOff, "Motor should be off after kill.");
  }
}
