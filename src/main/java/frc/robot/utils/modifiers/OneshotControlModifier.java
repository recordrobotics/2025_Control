package frc.robot.utils.modifiers;

public abstract class OneshotControlModifier implements IDrivetrainControlModifier {

    private boolean enableForThisPeriodic = false;

    @Override
    public final boolean isEnabled() {
        return enableForThisPeriodic;
    }

    @Override
    public final void setEnabled(boolean enable) {
        enableForThisPeriodic = enable;
    }

    @Override
    public final boolean apply(DrivetrainControl control) {
        enableForThisPeriodic = false;
        return perform(control);
    }

    /**
     * Apply the modifier to the control
     *
     * @param control
     * @return true if the modifier was applied, false otherwise
     */
    protected abstract boolean perform(DrivetrainControl control);
}
