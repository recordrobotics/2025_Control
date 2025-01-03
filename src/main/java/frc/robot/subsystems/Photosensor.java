package frc.robot.subsystems;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shuffleboard.ShuffleboardUI;

public class Photosensor extends SubsystemBase {

    private static DigitalInput photosensor;
    private static Boolean debounced_value = false;
    Debouncer m_debouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

    public Photosensor() {
        photosensor = new DigitalInput(0);
        ShuffleboardUI.Overview.setHasNote(this::getDebouncedValue);
        ShuffleboardUI.Autonomous.setHasNote(this::getDebouncedValue);
    }

    public void periodic() {
        debounced_value = !m_debouncer.calculate(getCurrentValue());
    }

    /**
     * Gets the debounced state of the photosensor, meaning that x time must pass
     * before the sensor returns true
     * (true = object detected, false = no object detected)
     */
    public Boolean getDebouncedValue() {
        return debounced_value;
    }

    /**
     * Gets the current state of the photosensor (true = object detected, false = no
     * object detected)
     */
    public Boolean getCurrentValue() {
        return photosensor.get();
    }
}