package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;

public class SwerveModuleStub implements SwerveModuleIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  public SwerveModuleStub(double periodicDt, ModuleConstants m) {
    this.periodicDt = periodicDt;
  }

  @Override
  public void applyDriveTalonFXConfig(TalonFXConfiguration configuration) {}

  @Override
  public void applyTurnTalonFXConfig(TalonFXConfiguration configuration) {}

  @Override
  public void setDriveMotorVoltage(double newValue) {}

  @Override
  public void setTurnMotorVoltage(double newValue) {}

  @Override
  public void setTurnMotorMotionMagic(MotionMagicVoltage request) {}

  @Override
  public void setDriveMotorMotionMagic(MotionMagicVelocityVoltage request) {}

  @Override
  public double getDriveMotorVoltage() {
    return 0;
  }

  @Override
  public double getTurnMotorVoltage() {
    return 0;
  }

  @Override
  public void setDriveMotorPercent(double newValue) {}

  @Override
  public void setTurnMotorPercent(double newValue) {}

  @Override
  public double getDriveMotorPercent() {
    return 0;
  }

  @Override
  public double getTurnMotorPercent() {
    return 0;
  }

  @Override
  public double getAbsoluteEncoder() {
    return 0;
  }

  @Override
  public double getTurnMotorPosition() {
    return 0;
  }

  @Override
  public double getTurnMotorVelocity() {
    return 0;
  }

  @Override
  public double getDriveMotorPosition() {
    return 0;
  }

  @Override
  public double getDriveMotorVelocity() {
    return 0;
  }

  @Override
  public void setDriveMotorPosition(double newValue) {}

  @Override
  public void setTurnMotorPosition(double newValue) {}

  @Override
  public void close() throws Exception {}

  @Override
  public double getDriveMotorCurrentDrawAmps() {
    return 0;
  }

  @Override
  public double getTurnMotorCurrentDrawAmps() {
    return 0;
  }

  @Override
  public void simulationPeriodic() {}
}
