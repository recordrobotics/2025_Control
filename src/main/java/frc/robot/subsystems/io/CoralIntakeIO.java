package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public interface CoralIntakeIO extends AutoCloseable {

  public void applyArmTalonFXConfig(TalonFXConfiguration configuration);

  public void setWheelVoltage(double outputVolts);

  public void setArmVoltage(double outputVolts);

  public void setWheelPosition(double newValue);

  public void setArmPosition(double newValue);

  public void setArmMotionMagic(MotionMagicExpoVoltage request);

  public double getWheelPosition();

  public double getWheelVelocity();

  public double getWheelVoltage();

  public double getArmVoltage();

  public double getArmPosition();

  public double getArmVelocity();

  public void setWheelPercent(double newValue);

  public void setArmPercent(double newValue);

  public double getWheelPercent();

  public double getArmPercent();

  public double getWheelCurrentDrawAmps();

  public double getArmCurrentDrawAmps();

  public void simulationPeriodic();
}
