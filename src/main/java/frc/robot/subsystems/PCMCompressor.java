package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shuffleboard.ShuffleboardUI;

public class PCMCompressor extends SubsystemBase implements ShuffleboardPublisher {
  private static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private static boolean isDisabledManually = false;

  public PCMCompressor() {}

  public void disable() {
    compressor.disable();
    isDisabledManually = true;
  }

  public void enable() {
    compressor.enableDigital();
    isDisabledManually = false;
  }

  public double getCurrent() {
    return compressor.getCurrent();
  }

  public boolean isEnabled() {
    try {
      return compressor.isEnabled();
    } catch (Exception e) { // Prevent disconnected compressor from crashing code
      return false;
    }
  }

  public boolean isDisabledManually() {
    return isDisabledManually;
  }

  public boolean isFull() {
    return compressor.getPressureSwitchValue();
  }

  public boolean isPumping() {
    return !this.isFull() && this.isEnabled() && !this.isDisabledManually();
  }

  @Override
  public void periodic() {}

  /** frees up all hardware allocations */
  public void close() {
    compressor.close();
  }

  @Override
  public void setupShuffleboard() {
    ShuffleboardUI.Overview.setCompressor(this::isEnabled);
    ShuffleboardUI.Overview.setCompressorManuallyDisabled(this::isDisabledManually);
  }
}
