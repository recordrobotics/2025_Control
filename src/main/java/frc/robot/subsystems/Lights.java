package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;
import java.util.function.Supplier;

public class Lights extends SubsystemBase implements AutoCloseable {
  public boolean RunningElevator = false;
  public boolean RunningAlgae = false;
  public boolean RunningCoralIntake = false;
  public boolean RunningCoralShooter = false;

  private AddressableLED LEDs;
  private AddressableLEDBuffer buffer;

  // TODO constants
  private static final LEDPattern pulsatingOrange =
      LEDPattern.solid(Color.kOrange)
          .breathe(Constants.Lights.pulsateFrequency)
          .blend(LEDPattern.solid(Color.kOrange));
  private static final LEDPattern pulsatingGreen =
      LEDPattern.solid(Color.kGreen).breathe(Constants.Lights.pulsateFrequency);

  private static final LEDPattern elevatorPattern =
      pulsatingGreen
          .mask(
              LEDPattern.progressMaskLayer(
                  () -> RobotContainer.elevator.getCurrentHeight() / Constants.Elevator.MAX_HEIGHT))
          .overlayOn(pulsatingOrange);
  private static final Supplier<LEDPattern> algaePattern =
      () ->
          LEDPattern.solid(
              Color.lerpRGB(
                  Color.kRed,
                  Color.kGreen,
                  (RobotContainer.groundAlgae.getArmAngle() - Constants.GroundAlgae.UP_ANGLE)
                      / (Constants.GroundAlgae.DOWN_ANGLE
                          - Constants.GroundAlgae
                              .UP_ANGLE))); // TODO this is some of the worst code ive seen today
  private static final Supplier<LEDPattern> coralIntakePattern =
      () ->
          LEDPattern.solid(
              Color.lerpRGB(
                  Color.kRed,
                  Color.kGreen,
                  (RobotContainer.coralIntake.getServoAngle() - Constants.CoralIntake.SERVO_UP)
                      / (Constants.CoralIntake.SERVO_DOWN
                          - Constants.CoralIntake
                              .SERVO_UP))); // TODO also bad (but not the worst ive written today)
  private static final Supplier<LEDPattern> coralShooterPattern =
      () ->
          (RobotContainer.coralShooter.getCurrentState() == CoralShooterStates.OFF)
              ? pulsatingGreen
              : pulsatingOrange;

  public Lights() {
    LEDs = new AddressableLED(RobotMap.Lights.LED_ID);
    buffer = new AddressableLEDBuffer(Constants.Lights.length);
    LEDs.setColorOrder(ColorOrder.kRGB);
    LEDs.setLength(Constants.Lights.length);
    LEDs.start();
    off();

    // Default command makes lights turn off by default
    setDefaultCommand(
        runPattern(LEDPattern.solid(Color.kBlack), 0, Constants.Lights.length - 1).withName("Off"));
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

    if (RunningElevator) {
      setRangePattern(
          elevatorPattern,
          Constants.Lights.PART_INDECIES.get("Elevator").getFirst(),
          Constants.Lights.PART_INDECIES.get("Elevator").getSecond());
    }
    if (RunningAlgae) {
      setRangePattern(
          algaePattern.get(),
          Constants.Lights.PART_INDECIES.get("Algae").getFirst(),
          Constants.Lights.PART_INDECIES.get("Algae").getSecond());
    }
    if (RunningCoralIntake) {
      setRangePattern(
          coralIntakePattern.get(),
          Constants.Lights.PART_INDECIES.get("CoralIntake").getFirst(),
          Constants.Lights.PART_INDECIES.get("CoralIntake").getSecond());
    }
    if (RunningCoralShooter) {
      setRangePattern(
          coralShooterPattern.get(),
          Constants.Lights.PART_INDECIES.get("CoralShooter").getFirst(),
          Constants.Lights.PART_INDECIES.get("CoralShooter").getSecond());
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
  public Command runPattern(LEDPattern pattern, int start, int end) {
    return run(() -> pattern.applyTo(buffer.createView(start, end)))
        .ignoringDisable(true); // TODO test view things
  }

  public void close() {
    LEDs.close();
  }
}
