package frc.robot.shuffleboard;

import java.util.ArrayList;
import java.util.EventListener;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TestLayout extends AbstractLayout {

    public interface PeriodicListener<T> extends EventListener {
        public void periodic(T value);
    }

    public class PeriodicNotifier<T> {
        private final List<PeriodicListener<T>> listeners = new ArrayList<PeriodicListener<T>>();

        public void subscribe(PeriodicListener<T> listener) {
            listeners.add(listener);
        }

        protected void invoke(T value) {
            for (PeriodicListener<T> listener : listeners) {
                listener.periodic(value);
            }
        }
    }
    
    private final Map<GenericEntry, PeriodicNotifier<Double>> sliderMap = new HashMap<>();
    private final Map<GenericEntry, PeriodicNotifier<Rotation2d>> headingMap = new HashMap<>();

    public <T extends MotorController & Sendable> void addMotor(String name, T motor) {
        getTab()
            .add(name, motor)
            .withWidget(BuiltInWidgets.kMotorController);
    }

    public void addBoolean(String name, BooleanSupplier value){
        getTab().addBoolean(name, value);
    }

    public PeriodicNotifier<Double> addSlider(String name, double value, double min, double max) {
        GenericEntry entry = getTab().add(name, value)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", min, "max", max))
                .getEntry();

        var notifier = new PeriodicNotifier<Double>();
        sliderMap.put(entry, notifier);
        return notifier;
    }

    public PeriodicNotifier<Rotation2d> addHeading(String name, Rotation2d rotation) {
        GenericEntry entry = getTab().add(name, rotation.getRadians())
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", Math.PI * 2))
                .getEntry();

        var notifier = new PeriodicNotifier<Rotation2d>();
        headingMap.put(entry, notifier);
        return notifier;
    }

    public void addNumber(String name, DoubleSupplier value){
        getTab().addDouble(name, value);
    }
    
    public void testPeriodic() {
        for (GenericEntry key : sliderMap.keySet()) {
            sliderMap.get(key).invoke(key.getDouble(0));
        }

        for (GenericEntry key : headingMap.keySet()) {
            headingMap.get(key).invoke(new Rotation2d(key.getDouble(0)));
        }
    }

    @Override
    public ShuffleboardTab getTab() {
        return Shuffleboard.getTab("Test");
    }
}