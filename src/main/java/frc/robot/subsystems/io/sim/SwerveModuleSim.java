package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.io.SwerveModuleIO;
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

    dcDriveMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(dcDriveMotor, 0.01, DRIVE_GEAR_RATIO),
            dcDriveMotor,
            0.001,
            0.001);

    dcTurnMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(dcTurnMotor, 0.001, TURN_GEAR_RATIO),
            dcDriveMotor,
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
  public void setTurnMotorMotionMagic(MotionMagicVoltage request) {
    m_turningMotor.setControl(request);
  }

  @Override
  public void setDriveMotorMotionMagic(MotionMagicVelocityVoltage request) {
    m_driveMotor.setControl(request);
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
    if (m_driveMotorSim.Orientation == ChassisReference.Clockwise_Positive)
      return m_driveMotor.getSupplyCurrent().getValueAsDouble();
    else return -m_driveMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public double getTurnMotorCurrentDrawAmps() {
    if (m_turningMotorSim.Orientation == ChassisReference.Clockwise_Positive)
      return m_turningMotor.getSupplyCurrent().getValueAsDouble();
    else return -m_turningMotor.getSupplyCurrent().getValueAsDouble();
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

    var driveMotorVoltage = m_driveMotorSim.getMotorVoltageMeasure();
    var turnMotorVoltage = m_turningMotorSim.getMotorVoltageMeasure();

    dcDriveMotorSim.setInputVoltage(driveMotorVoltage.in(Volts));
    dcDriveMotorSim.update(periodicDt);

    dcTurnMotorSim.setInputVoltage(turnMotorVoltage.in(Volts));
    dcTurnMotorSim.update(periodicDt);

    m_driveMotorSim.setRawRotorPosition(
        dcDriveMotorSim.getAngularPosition().times(DRIVE_GEAR_RATIO));
    m_driveMotorSim.setRotorVelocity(dcDriveMotorSim.getAngularVelocity().times(DRIVE_GEAR_RATIO));

    m_turningMotorSim.setRawRotorPosition(
        dcTurnMotorSim.getAngularPosition().times(TURN_GEAR_RATIO));
    m_turningMotorSim.setRotorVelocity(dcTurnMotorSim.getAngularVelocity().times(TURN_GEAR_RATIO));
  }
}
