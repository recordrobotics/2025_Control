package frc.robot.utils.modifiers;

public interface IDrivetrainControlModifier {

    /**
     * Apply the modifier to the given drivetrain control input.
     *
     * @param control the DrivetrainControl object representing the current control input to be modified
     * @return true if the modifier was applied, false otherwise
     */
    boolean apply(DrivetrainControl control);

    boolean isEnabled();

    void setEnabled(boolean enabled);
}
