package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.lights.AlgaeGrabberLights;
import frc.robot.subsystems.lights.CoralIntakeLights;
import frc.robot.subsystems.lights.CoralShooterLights;
import frc.robot.subsystems.lights.ElevatorLights;
import frc.robot.subsystems.lights.StateVisualizerLights;

public class Lights extends SubsystemBase implements AutoCloseable {

  public AlgaeGrabberLights algaeGrabber;
  public CoralIntakeLights coralIntake;
  public CoralShooterLights coralShooter;
  public ElevatorLights elevator;
  public StateVisualizerLights stateVisualizer;

  private AddressableLED LEDs;
  private AddressableLEDBuffer buffer;

  public Lights() {
    LEDs = new AddressableLED(RobotMap.Lights.LED_ID);
    buffer = new AddressableLEDBuffer(Constants.Lights.LENGTH);
    LEDs.setColorOrder(ColorOrder.kRGB);
    LEDs.setLength(Constants.Lights.LENGTH);
    LEDs.start();

    algaeGrabber = new AlgaeGrabberLights(this);
    coralIntake = new CoralIntakeLights(this);
    coralShooter = new CoralShooterLights(this);
    elevator = new ElevatorLights(this);
    stateVisualizer = new StateVisualizerLights(this);
  }

  public void periodicManaged() {
    // Send the latest LED color data to the LED strip
    LEDs.setData(buffer);
  }

  public AddressableLEDBuffer getBuffer() {
    return buffer;
  }

  public void close() {
    LEDs.close();
  }
}
