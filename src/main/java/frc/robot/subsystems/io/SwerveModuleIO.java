package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public interface SwerveModuleIO extends AutoCloseable {

  public void applyDriveTalonFXConfig(TalonFXConfiguration configuration);

  public void applyTurnTalonFXConfig(TalonFXConfiguration configuration);

  public void setDriveMotorVoltage(double newValue);

  public void setTurnMotorVoltage(double newValue);

  public double getDriveMotorVoltage();

  public double getTurnMotorVoltage();

  public void setDriveMotorPercent(double newValue);

  public void setTurnMotorPercent(double newValue);

  public double getDriveMotorPercent();

  public double getTurnMotorPercent();

  public double getAbsoluteEncoder();

  public double getTurnMotorPosition();

  public double getTurnMotorVelocity();

  public double getDriveMotorPosition();

  public double getDriveMotorVelocity();

  public void setDriveMotorPosition(double newValue);

  public void setTurnMotorPosition(double newValue);

  public double getDriveMotorCurrentDrawAmps();

  public double getTurnMotorCurrentDrawAmps();

  public void simulationPeriodic();
}
