package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface ElevatorArmIO extends AutoCloseable {

  public void applyArmTalonFXConfig(TalonFXConfiguration configuration);

  public void setArmVoltage(double outputVolts);

  public void setArmPosition(double newValue);

  public void setArmMotionMagic(MotionMagicVoltage request);

  public double getArmPosition();

  public double getArmVelocity();

  public void setArmPercent(double newValue);

  public double getArmPercent();

  public double getArmVoltage();

  public double getArmCurrentDrawAmps();

  public void simulationPeriodic();
}
