package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Lights extends SubsystemBase implements AutoCloseable {
  public enum LightMode {
    OFF,
    RUNNING,
    SUCCESS,
    FAIL,
    RAINBOW,
    CHASE
  }

  private AddressableLED LEDs;
  private AddressableLEDBuffer buffer;

  public Lights() {
    LEDs = new AddressableLED(RobotMap.Lights.LED_ID);
    buffer = new AddressableLEDBuffer(Constants.Lights.length);
    LEDs.setColorOrder(ColorOrder.kRGB);
    LEDs.setLength(Constants.Lights.length);
    LEDs.start();
    setGlobalMode(LightMode.OFF);

    // Default command makes lights turn off by default
    setDefaultCommand(
        runPattern(LEDPattern.solid(Color.kBlack), 0, Constants.Lights.length - 1).withName("Off"));
  }

  /** NOTE: Only interact with light strip with the command!!! */
  public void setGlobalMode(LightMode mode) {
    setRangeMode(mode, 0, Constants.Lights.length - 1);
  }

  /** NOTE: Only interact with light strip with the command!!! */
  /**
   * @param mode the mode to set the LED strip to
   * @param start the start index of the LED strip to run the pattern on (inclusive)
   * @param end the end index of the LED strip to run the pattern on (inclusive)
   */
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
      case RAINBOW:
        pattern = LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of((25)));
        break;
      case CHASE:
        pattern =
            LEDPattern.rainbow(255, 255)
                .mask(
                    LEDPattern.progressMaskLayer(() -> 0.2)
                        .scrollAtRelativeSpeed(Percent.per(Second).of((105))))
                .scrollAtRelativeSpeed(Percent.per(Second).of((-30)));
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
   * @param start the start index of the LED strip to run the pattern on (inclusive)
   * @param end the end index of the LED strip to run the pattern on (inclusive)
   */
  public Command runPattern(LEDPattern pattern, int start, int end) {
    return run(() -> pattern.applyTo(buffer.createView(start, end)))
        .ignoringDisable(true); // TODO test view things
  }

  public void close() {
    LEDs.close();
  }
}
