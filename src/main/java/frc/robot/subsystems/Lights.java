package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Lights.LightSegments;
import frc.robot.RobotMap;
import java.util.Map;
import java.util.function.Supplier;

public class Lights extends SubsystemBase implements AutoCloseable {
  public Map<LightSegments, Supplier<LEDPattern>> patterns = Constants.Lights.DEFAULT_PATTERNS;

  private AddressableLED LEDs;
  private AddressableLEDBuffer buffer;

  public Lights() {
    LEDs = new AddressableLED(RobotMap.Lights.LED_ID);
    buffer = new AddressableLEDBuffer(Constants.Lights.length);
    LEDs.setColorOrder(ColorOrder.kRGB);
    LEDs.setLength(Constants.Lights.length);
    LEDs.start();
    off();

    // Default command makes lights turn off by default
    setDefaultCommand(new InstantCommand(this::off));
  }

  private void setGlobalPattern(LEDPattern pattern) {
    setRangePattern(pattern, 0, Constants.Lights.length - 1);
  }

  /**
   * @param mode the mode to set the LED strip to
   * @param start the start index of the LED strip to run the pattern on (inclusive)
   * @param end the end index of the LED strip to run the pattern on (inclusive)
   */
  private void setRangePattern(LEDPattern pattern, int start, int end) {
    runPattern(pattern.atBrightness(Constants.Lights.multiplier), start, end).schedule();
  }

  public void off() {
    setGlobalPattern(LEDPattern.solid(Color.kBlack));
  }

  public void periodic() {
    off(); // Clear the LED strip

    // loop through LightSegments
    for (Map.Entry<LightSegments, Supplier<LEDPattern>> entry : patterns.entrySet()) {
      setRangePattern( // TODO get the getValue get getkey getFirst AAAAAAAAAAAAAAAAAAAAAAA
          entry.getValue().get(),
          Constants.Lights.PART_INDECIES.get(entry.getKey()).getFirst(),
          Constants.Lights.PART_INDECIES.get(entry.getKey()).getSecond());
    }

    // Send the latest LED color data to the LED strip
    LEDs.setData(buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   * @param start the start index of the LED strip to run the pattern on (inclusive)
   * @param end the end index of the LED strip to run the pattern on (inclusive)
   */
  private Command runPattern(LEDPattern pattern, int start, int end) {
    return run(() -> pattern.applyTo(buffer.createView(start, end)))
        .ignoringDisable(true); // TODO test view things
  }

  public void close() {
    LEDs.close();
  }
}
