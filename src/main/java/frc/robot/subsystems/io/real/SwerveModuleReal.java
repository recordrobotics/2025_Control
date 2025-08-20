package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.io.SwerveModuleIO;
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
    public void setTurnMotorMotionMagic(MotionMagicExpoVoltage request) {
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
    public double getDriveMechanismAcceleration() {
        return m_driveMotor.getAcceleration().getValueAsDouble();
    }

    @Override
    public void setDriveMechanismPosition(double newValue) {
        m_driveMotor.setPosition(newValue);
    }

    @Override
    public void setTurnMechanismPosition(double newValue) {
        m_turningMotor.setPosition(newValue);
    }

    @Override
    public void close() throws Exception {
        m_driveMotor.close();
        m_turningMotor.close();
        absoluteTurningMotorEncoder.close();
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
    public void simulationPeriodic() {}
}
