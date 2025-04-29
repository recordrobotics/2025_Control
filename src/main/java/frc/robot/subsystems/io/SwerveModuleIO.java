package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public interface SwerveModuleIO extends AutoCloseable {

  public void applyDriveTalonFXConfig(TalonFXConfiguration configuration);

  public void applyTurnTalonFXConfig(TalonFXConfiguration configuration);

  public void setDriveMotorVoltage(double newValue);

  public void setTurnMotorVoltage(double newValue);

  public void setTurnMotorMotionMagic(MotionMagicVoltage request);

  public void setDriveMotorMotionMagic(MotionMagicVelocityVoltage request);

  public double getDriveMotorVoltage();

  public double getTurnMotorVoltage();

  public void setDriveMotorPercent(double newValue);

  public void setTurnMotorPercent(double newValue);

  public double getDriveMotorPercent();

  public double getTurnMotorPercent();

  public double getAbsoluteEncoder();

  public double getTurnMechanismPosition();

  public double getTurnMechanismVelocity();

  public double getDriveMechanismPosition();

  public double getDriveMechanismVelocity();

  public void setDriveMechanismPosition(double newValue);

  public void setTurnMechanismPosition(double newValue);

  public double getDriveMotorCurrentDrawAmps();

  public double getTurnMotorCurrentDrawAmps();

  public void simulationPeriodic();
}
