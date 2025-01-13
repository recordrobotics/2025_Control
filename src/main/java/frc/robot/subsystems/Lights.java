package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {
  public enum LightMode {
    OFF,
    RUNNING,
    SUCCESS,
    FAIL
  }

  private AddressableLED LEDs;
  private AddressableLEDBuffer buffer;

  public Lights() {
    LEDs = new AddressableLED(-1); // TODO do port stuff
    buffer = new AddressableLEDBuffer(Constants.Lights.length);
    LEDs.setLength(Constants.Lights.length);
    LEDs.start();
    setGlobalMode(LightMode.OFF);

    // Default command makes lights turn off by default
    setDefaultCommand(
        runPattern(LEDPattern.solid(Color.kBlack), 0, Constants.Lights.length).withName("Off"));
  }

  /** NOTE: Only interact with light strip with the command!!! */
  public void setGlobalMode(LightMode mode) {
    setRangeMode(mode, 0, Constants.Lights.length);
  }

  /** NOTE: Only interact with light strip with the command!!! */
  public void setRangeMode(LightMode mode, int start, int end) {
    LEDPattern pattern;

    switch (mode) {
      case RUNNING:
        pattern = LEDPattern.solid(Color.kOrange); // Just orange
        pattern =
            pattern.breathe(Seconds.of(Constants.Lights.pulsateFrequency)); // Pulsating orange
        pattern = pattern.blend(LEDPattern.solid(Color.kOrange)); // Prevent from going to 0
        break;
      case SUCCESS:
        pattern = LEDPattern.solid(Color.kGreen);
        break;
      case FAIL:
        pattern = LEDPattern.solid(Color.kRed);
        break;
      case OFF:
      default:
        pattern = LEDPattern.solid(Color.kBlack);
        break;
    }

    runPattern(pattern.atBrightness(Percent.of(Constants.Lights.multiplier)), start, end)
        .schedule();
  }

  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    LEDs.setData(buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern, int start, int end) {
    return run(() -> pattern.applyTo(buffer.createView(start, end))); // TODO test view things
  }
}
