package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StrictFollower;

public interface ElevatorIO extends AutoCloseable {

  public void applyTalonFXConfig(TalonFXConfiguration configuration);

  public StrictFollower createFollower();

  public void setLeadMotorVoltage(double outputVolts);

  public double getLeadMotorVoltage();

  public double getFollowerMotorVoltage();

  public void setLeadMotorPosition(double newValue);

  public void setFollowerMotorPosition(double newValue);

  public void setLeadMotionMagic(MotionMagicExpoVoltage request);

  public void setFollowerMotionMagic(StrictFollower request);

  public double getLeadMotorPosition();

  public double getLeadMotorVelocity();

  public double getFollowerMotorPosition();

  public double getFollowerMotorVelocity();

  public void setLeadMotorPercent(double newValue);

  public double getLeadMotorPercent();

  public boolean getTopEndStop();

  public boolean getBottomEndStop();

  public double getLeadMotorCurrentDraw();

  public double getFollowerMotorCurrentDraw();

  public void simulationPeriodic();
}
