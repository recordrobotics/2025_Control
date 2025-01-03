import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Channel;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ChannelTest {
  private Channel channel;
  private PWMSim motorSim;
  private static final double TOLERANCE = 0.01;

  @BeforeEach
  public void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    channel = new Channel();
    motorSim = new PWMSim(RobotMap.Channel.CHANNEL_MOTOR_ID); // Simulate the motor with PWMSim
  }

  @AfterEach
  public void shutdown() throws Exception {
    channel.close();
  }

  @Test
  public void testInitialState() {
    assertEquals(0, motorSim.getSpeed(), TOLERANCE, "Motor should be off initially.");
  }

  @Test
  public void testToggleThroughState() {
    channel.toggle(Channel.ChannelStates.THROUGH);
    assertEquals(
        Constants.Channel.THROUGH_SPEED,
        motorSim.getSpeed(),
        TOLERANCE,
        "Motor should be set to default THROUGH speed.");
  }

  @Test
  public void testToggleReverseState() {
    channel.toggle(Channel.ChannelStates.REVERSE);
    assertEquals(
        Constants.Channel.REVERSE_SPEED,
        motorSim.getSpeed(),
        TOLERANCE,
        "Motor should be set to default REVERSE speed.");
  }

  @Test
  public void testToggleShootState() {
    channel.toggle(Channel.ChannelStates.SHOOT);
    assertEquals(
        Constants.Channel.SHOOT_SPEED,
        motorSim.getSpeed(),
        TOLERANCE,
        "Motor should be set to default SHOOT speed.");
  }

  @Test
  public void testToggleCustomState() {
    channel.toggle(1.5);
    assertEquals(1, motorSim.getSpeed(), TOLERANCE, "Motor should not go above 1 speed");
  }

  @Test
  public void testToggleNegativeCustomState() {
    channel.toggle(-1.5);
    assertEquals(-1, motorSim.getSpeed(), TOLERANCE, "Motor should not go below -1 speed");
  }

  @Test
  public void testToggleOffState() {
    channel.toggle(Channel.ChannelStates.THROUGH); // Set to IN first
    channel.toggle(Channel.ChannelStates.OFF); // Now set to OFF
    assertEquals(0, motorSim.getSpeed(), TOLERANCE, "Motor should be off when state is OFF.");
  }

  @Test
  public void testKill() {
    channel.toggle(Channel.ChannelStates.THROUGH); // Activate the motor
    channel.kill(); // Kill should turn off the motor
    assertEquals(0, motorSim.getSpeed(), TOLERANCE, "Motor should be off after kill.");
  }
}
