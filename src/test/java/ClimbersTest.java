import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Climbers.ClimberStates;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ClimbersTest {
  private Climbers climbers;
  private DoubleSolenoidSim pistonSim;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    climbers = new Climbers();
    pistonSim =
        new DoubleSolenoidSim(
            PneumaticsModuleType.CTREPCM,
            RobotMap.Climbers.FORWARD_PORT,
            RobotMap.Climbers.REVERSE_PORT); // Simulate the piston with DoubleSolenoidSim
  }

  @AfterEach
  public void shutdown() throws Exception {
    climbers.close();
  }

  private void check(Value correct, String msg) {
    assertEquals(correct, pistonSim.get(), msg);
  }

  @Test
  public void testInitialState() {
    check(Value.kOff, "Motor should be off initially.");
  }

  @Test
  public void testToggleUpState() {
    climbers.toggle(ClimberStates.UP);
    check(Value.kForward, "Motor should be set to UP.");
  }

  @Test
  public void testToggleDownState() {
    climbers.toggle(ClimberStates.DOWN);
    check(Value.kReverse, "Motor should be set to DOWN.");
  }

  @Test
  public void testToggleOffState() {
    climbers.toggle(ClimberStates.UP); // Set to UP first
    climbers.toggle(ClimberStates.OFF); // Now set to OFF
    check(Value.kOff, "Motor should be off when state is OFF.");
  }

  @Test
  public void testKill() {
    climbers.toggle(ClimberStates.UP); // Set to UP first
    climbers.kill(); // Kill should turn it off
    check(Value.kOff, "Motor should be off after kill.");
  }
}
