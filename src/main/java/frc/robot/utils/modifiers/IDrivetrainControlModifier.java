package frc.robot.utils.modifiers;

public interface IDrivetrainControlModifier {

    /**
     * Apply the modifier to the control
     *
     * @param control
     * @return true if the modifier was applied, false otherwise
     */
    boolean apply(DrivetrainControl control);

    boolean isEnabled();

    void setEnabled(boolean enabled);
}
