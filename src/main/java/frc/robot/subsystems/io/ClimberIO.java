package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface ClimberIO extends AutoCloseable {
  public void applyTalonFXConfig(TalonFXConfiguration configuration);

  public void setVoltage(double outputVolts);

  public void setPosition(double newValue);

  public void setMotionMagic(MotionMagicVoltage request);

  public double getPosition();

  public double getVelocity();

  public void setPercent(double newValue);

  public double getPercent();

  public double getVoltage();

  public void setRatchet(double value);

  public double getCurrentDrawAmps();

  public void simulationPeriodic();
}
