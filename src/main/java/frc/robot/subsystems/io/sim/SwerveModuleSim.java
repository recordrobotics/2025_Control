package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class SwerveModuleSim implements SwerveModuleIO {

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final DutyCycleEncoder absoluteTurningMotorEncoder;

  private final TalonFXSimState m_driveMotorSim;
  private final TalonFXSimState m_turningMotorSim;

  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    public final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.id = talonFX.getDeviceID();
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  public SwerveModuleSim(SwerveModuleSimulation moduleSimulation, ModuleConstants m) {
    m_driveMotor = new TalonFX(m.driveMotorChannel);
    m_turningMotor = new TalonFX(m.turningMotorChannel);

    absoluteTurningMotorEncoder = new DutyCycleEncoder(m.absoluteTurningMotorEncoderChannel);

    m_driveMotorSim = m_driveMotor.getSimState();
    m_turningMotorSim = m_turningMotor.getSimState();

    m_turningMotorSim.Orientation = ChassisReference.Clockwise_Positive;

    moduleSimulation.useDriveMotorController(new TalonFXMotorControllerSim(m_driveMotor));
    moduleSimulation.useSteerMotorController(new TalonFXMotorControllerSim(m_turningMotor));
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
  public double getTurnMechanismPosition() {
    return m_turningMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getTurnMechanismVelocity() {
    return m_turningMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public double getDriveMechanismPosition() {
    return m_driveMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getDriveMechanismVelocity() {
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setDriveMechanismPosition(double newValue) {
    // m_driveMotor.setPosition(newValue);
  }

  @Override
  public void setTurnMechanismPosition(double newValue) {
    // m_turningMotor.setPosition(newValue);
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
  public void simulationPeriodic() {}
}
