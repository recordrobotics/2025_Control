package frc.robot.utils.simulation;

import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class CANSparkMaxSim {
  private double currentSpeed;
  private static final double minSpeed = -1;
  private static final double maxSpeed = 1;

  public CANSparkMaxSim() {}

  private double clamp(double old) {
    if (old < minSpeed) {
      return minSpeed;
    } else if (old > maxSpeed) {
      return maxSpeed;
    } else {
      return old;
    }
  }

  public void set(double speed) {
    currentSpeed = clamp(speed);
  }

  /**
   * Returns the current speed of the motor in RPM
   *
   * @return the current motor speed in RPM
   */
  public double get() {
    return currentSpeed * 12 * Constants.NeoSim.NEO_MOTOR_KV;
  }

  public RelativeEncoder getEncoder() {
    return new CANSparkMaxEncoderSim(get(), 0);
  }

  public void setVoltage(double voltage) {
    set(voltage / 12);
  }
}
