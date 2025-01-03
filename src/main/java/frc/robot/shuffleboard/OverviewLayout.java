package frc.robot.shuffleboard;

import java.util.EnumSet;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.control.AbstractControl;

public class OverviewLayout extends AbstractLayout {

    public enum DriverOrientation {
        XAxisTowardsTrigger("Competition"),
        YAxisTowardsTrigger("Y Axis")
        ;

        public final String display_name;

        DriverOrientation(String orientation) {
            display_name = orientation;
        }
    }

    private static SendableChooser<DriverOrientation> driverOrientation = new SendableChooser<DriverOrientation>();
    private static SendableChooser<AbstractControl> driveMode = new SendableChooser<AbstractControl>();
    private static AbstractControl _defaultControl;

    //private Supplier<Boolean> poseCertainValue = () -> false;
    private Supplier<Boolean> acquisitionValue =  () -> false;
    private Supplier<Boolean> compressorValue =  () -> false;
    private Supplier<Boolean> hasNoteValue =  () -> false;
    private Supplier<Boolean> navSensorValue =  () -> false;

    public OverviewLayout() {
        // getTab()
        //         .addBoolean("Pose Certain", () -> poseCertainValue.get())
        //         .withWidget(BuiltInWidgets.kBooleanBox)
        //         .withSize(1, 1)
        //         .withPosition(0, 1);

        getTab()
                .addBoolean("Acquisition", () -> acquisitionValue.get())
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(7, 0)
                .withSize(1, 1);

        getTab()
                .addBoolean("Compressor", () -> compressorValue.get())
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(8, 0)
                .withSize(1, 1);

        getTab()
                .addBoolean("Has Note", () -> hasNoteValue.get())
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(9, 0)
                .withSize(1, 1);

        getTab()
                .addBoolean("Nav Sensor", () -> navSensorValue.get())
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(5, 1)
                .withSize(1, 1);
    }

    /**
     * Initializes the control object
     * @param defaultControl the first term will always be the default control object
     * @param controls any other control objects you want to initialize
     */
    public void addControls(AbstractControl defaultControl, AbstractControl... controls){
        _defaultControl = defaultControl;

        // Sets up drive mode options
        for (AbstractControl abstractControl : controls) {
            driveMode.addOption(abstractControl.getClass().getSimpleName(), abstractControl);
        }
        driveMode.setDefaultOption(defaultControl.getClass().getSimpleName(), defaultControl);

        EnumSet.allOf(DriverOrientation.class)
            .forEach(v -> driverOrientation.addOption(v.display_name, v));
        driverOrientation.setDefaultOption(DriverOrientation.XAxisTowardsTrigger.display_name, DriverOrientation.XAxisTowardsTrigger);

        // Creates the UI for driverOrientation
        getTab()
            .add("Driver Orientation", driverOrientation)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withPosition(0, 0)
            .withSize(2, 1);

        // Creates the UI for drive mode
        getTab()
            .add("Drive Mode", driveMode)
            .withWidget(BuiltInWidgets.kSplitButtonChooser)
            .withPosition(0, 1)
            .withSize(3, 1);
    }

    public void setPoseCertain(Supplier<Boolean> certain) {
        //poseCertainValue = certain;
    }

    public void setAcquisition(Supplier<Boolean> acquisition) {
        acquisitionValue = acquisition;
    }

    public void setCompressor(Supplier<Boolean> compressor) {
        compressorValue = compressor;
    }

    public void setHasNote(Supplier<Boolean> hasNote) {
        hasNoteValue = hasNote;
    }

    public void setNavSensor(Supplier<Boolean> navSensor) {
        navSensorValue = navSensor;
    }

    @Override
    public ShuffleboardTab getTab() {
        return Shuffleboard.getTab("Overview");
    }

    public DriverOrientation getDriverOrientation() {
        return driverOrientation.getSelected();
    }

    public AbstractControl getControl() {
        if(driveMode.getSelected() == null)
            return _defaultControl;
        return driveMode.getSelected();
    }
}