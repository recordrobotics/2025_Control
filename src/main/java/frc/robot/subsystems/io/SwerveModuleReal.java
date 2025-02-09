package frc.robot.subsystems.io;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.ModuleConstants;

public class SwerveModuleReal implements SwerveModuleIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final DutyCycleEncoder absoluteTurningMotorEncoder;

  public SwerveModuleReal(double periodicDt, ModuleConstants m) {
    this.periodicDt = periodicDt;

    m_driveMotor = new TalonFX(m.driveMotorChannel);
    m_turningMotor = new TalonFX(m.turningMotorChannel);

    absoluteTurningMotorEncoder = new DutyCycleEncoder(m.absoluteTurningMotorEncoderChannel);
  }

  @Override
  public void setDriveMotorVoltage(double newValue) {
    m_driveMotor.setVoltage(newValue);
  }

  @Override
  public void setTurnMotorVoltage(double newValue) {
    m_turningMotor.setVoltage(newValue);
  }

  @Override
  public double getDriveMotorVoltage() {
    return m_driveMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public double getTurnMotorVoltage() {
    return m_turningMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setDriveMotorPercent(double newValue) {
    m_driveMotor.set(newValue);
  }

  @Override
  public void setTurnMotorPercent(double newValue) {
    m_turningMotor.set(newValue);
  }

  @Override
  public double getDriveMotorPercent() {
    return m_driveMotor.get();
  }

  @Override
  public double getTurnMotorPercent() {
    return m_turningMotor.get();
  }

  @Override
  public double getAbsoluteEncoder() {
    return absoluteTurningMotorEncoder.get();
  }

  @Override
  public double getTurnMotorPosition() {
    return m_turningMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getTurnMotorVelocity() {
    return m_turningMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public double getDriveMotorPosition() {
    return m_driveMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getDriveMotorVelocity() {
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setDriveMotorPosition(double newValue) {
    m_driveMotor.setPosition(newValue);
  }

  @Override
  public void setTurnMotorPosition(double newValue) {
    m_turningMotor.setPosition(newValue);
  }

  @Override
  public void close() throws Exception {
    m_driveMotor.close();
    m_turningMotor.close();
    absoluteTurningMotorEncoder.close();
  }

  @Override
  public void simulationPeriodic() {}
}
