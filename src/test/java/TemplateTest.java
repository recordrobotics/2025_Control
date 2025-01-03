import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.ExampleSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TemplateTest {
  private ExampleSubsystem example;
  private PWMSim motorSim;
  private static final double TOLERANCE = 0.01;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    example = new ExampleSubsystem();
    motorSim =
        new PWMSim(RobotMap.ExampleSubsystem.MOTOR_ID); // Simulate the motor with PWMSim
  }

  @AfterEach
  public void shutdown() throws Exception {
    example.close();
  }

  @Test
  public void testInitialState() {
    assertEquals(
        ExampleSubsystem.ExampleSubsystemStates.OFF,
        example.getExampleSubsystemState(),
        "Initial state should be OFF.");
    assertEquals(0, motorSim.getSpeed(), TOLERANCE, "Motor should be off initially.");
  }

  @Test
  public void testToggleInState() {
    example.toggle(ExampleSubsystem.ExampleSubsystemStates.IN);
    assertEquals(
        ExampleSubsystem.ExampleSubsystemStates.IN, example.getExampleSubsystemState(), "State should be IN.");
    assertEquals(
        Constants.ExampleSubsystem.SPEED,
        motorSim.getSpeed(),
        TOLERANCE,
        "Motor should be set to default IN speed.");
  }

  @Test
  public void testKill() {
    example.toggle(ExampleSubsystem.ExampleSubsystemStates.IN); // Activate the motor
    example.kill(); // Kill should turn off the motor
    assertEquals(
        ExampleSubsystem.ExampleSubsystemStates.OFF,
        example.getExampleSubsystemState(),
        "Kill should set state to OFF.");
    assertEquals(0, motorSim.getSpeed(), TOLERANCE, "Motor should be off after kill.");
  }
}
