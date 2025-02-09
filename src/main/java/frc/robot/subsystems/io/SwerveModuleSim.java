package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.ModuleConstants;

public class SwerveModuleSim implements SwerveModuleIO {

  private final double periodicDt;

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final DutyCycleEncoder absoluteTurningMotorEncoder;

  private final TalonFXSimState m_driveMotorSim;
  private final TalonFXSimState m_turningMotorSim;

  private final DCMotor dcDriveMotor;
  private final DCMotor dcTurnMotor;

  private final DCMotorSim dcDriveMotorSim;
  private final DCMotorSim dcTurnMotorSim;

  private final double TURN_GEAR_RATIO;
  private final double DRIVE_GEAR_RATIO;

  public SwerveModuleSim(double periodicDt, ModuleConstants m) {
    this.periodicDt = periodicDt;

    m_driveMotor = new TalonFX(m.driveMotorChannel);
    m_turningMotor = new TalonFX(m.turningMotorChannel);

    TURN_GEAR_RATIO = m.TURN_GEAR_RATIO;
    DRIVE_GEAR_RATIO = m.DRIVE_GEAR_RATIO;

    absoluteTurningMotorEncoder = new DutyCycleEncoder(m.absoluteTurningMotorEncoderChannel);

    switch (m.driveMotorType) {
      case Falcon:
        dcDriveMotor = DCMotor.getFalcon500(1);
        break;
      case Kraken:
        dcDriveMotor = DCMotor.getKrakenX60(1);
        break;
      default:
        dcDriveMotor = DCMotor.getFalcon500(1);
        break;
    }

    switch (m.turnMotorType) {
      case Falcon:
        dcTurnMotor = DCMotor.getFalcon500(1);
        break;
      case Kraken:
        dcTurnMotor = DCMotor.getKrakenX60(1);
        break;
      default:
        dcTurnMotor = DCMotor.getFalcon500(1);
        break;
    }

    // TODO: fix divide by 19.4 workaround (why does this fix the motor sim?)
    dcDriveMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(m.DRIVE_KV / 19.4, m.DRIVE_KA / 19.4),
            dcDriveMotor.withReduction(DRIVE_GEAR_RATIO),
            0.001,
            0.001);

    // TODO: fix divide by 8 workaround
    dcTurnMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(m.TURN_KV / 8, m.TURN_KA / 8),
            dcTurnMotor.withReduction(TURN_GEAR_RATIO),
            0.001,
            0.001);

    m_driveMotorSim = m_driveMotor.getSimState();
    m_turningMotorSim = m_turningMotor.getSimState();

    m_turningMotorSim.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void applyDriveTalonFXConfig(TalonFXConfiguration configuration) {
    m_driveMotor.getConfigurator().apply(configuration);
  }

  @Override
  public void applyTurnTalonFXConfig(TalonFXConfiguration configuration) {
    m_turningMotor.getConfigurator().apply(configuration);
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
  public double getDriveMotorCurrentDrawAmps() {
    return m_driveMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public double getTurnMotorCurrentDrawAmps() {
    return m_turningMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    m_driveMotor.close();
    m_turningMotor.close();
    absoluteTurningMotorEncoder.close();
  }

  @Override
  public void simulationPeriodic() {
    m_driveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_turningMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var driveMotorVoltage = m_driveMotorSim.getMotorVoltage();
    var turnMotorVoltage = m_turningMotorSim.getMotorVoltage();

    dcDriveMotorSim.setInputVoltage(driveMotorVoltage);
    dcDriveMotorSim.update(periodicDt);

    dcTurnMotorSim.setInputVoltage(turnMotorVoltage);
    dcTurnMotorSim.update(periodicDt);

    m_driveMotorSim.setRawRotorPosition(
        dcDriveMotorSim.getAngularPositionRotations() * DRIVE_GEAR_RATIO);
    m_driveMotorSim.setRotorVelocity(
        Units.radiansToRotations(dcDriveMotorSim.getAngularVelocityRadPerSec()) * DRIVE_GEAR_RATIO);

    m_turningMotorSim.setRawRotorPosition(
        dcTurnMotorSim.getAngularPositionRotations() * TURN_GEAR_RATIO);
    m_turningMotorSim.setRotorVelocity(
        Units.radiansToRotations(dcTurnMotorSim.getAngularVelocityRadPerSec()) * TURN_GEAR_RATIO);
  }
}
