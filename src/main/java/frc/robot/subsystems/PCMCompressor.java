package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardUI;

public class PCMCompressor extends SubsystemBase {
    // Creates AHRS _nav object
    private static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public PCMCompressor() {
        var widget = ShuffleboardUI.Overview.getTab().addBoolean("Compressor", compressor::isEnabled);
        widget.withWidget(BuiltInWidgets.kBooleanBox);
        widget.withPosition(8, 0);
        widget.withSize(1, 1);
    }

    public void disable() {
        compressor.disable();
    }

    public void enable() {
        compressor.enableDigital();
    }

    public double getCurrent() {
        return compressor.getCurrent();
    }

    public boolean isEnabled() {
        return compressor.isEnabled();
    }

    public boolean isPumping() {
        return compressor.getPressureSwitchValue();
    }

    @Override
    public void periodic() {
    }
}