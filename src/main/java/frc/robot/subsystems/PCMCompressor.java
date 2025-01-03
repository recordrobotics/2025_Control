package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shuffleboard.ShuffleboardUI;

public class PCMCompressor extends SubsystemBase {
    private static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public PCMCompressor() {
        ShuffleboardUI.Overview.setCompressor(this::isEnabled);
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
        try{
            return compressor.isEnabled();
        } catch(Exception e) {
            return false;
        }
    }

    public boolean isPumping() {
        return compressor.getPressureSwitchValue();
    }

    @Override
    public void periodic() {
    }
}