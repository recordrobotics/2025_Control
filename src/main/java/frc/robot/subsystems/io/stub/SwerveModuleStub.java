package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;

public class SwerveModuleStub implements SwerveModuleIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    @SuppressWarnings("java:S1172")
    public SwerveModuleStub(double periodicDt, ModuleConstants m) {
        this.periodicDt = periodicDt;
    }

    @Override
    public void applyDriveTalonFXConfig(TalonFXConfiguration configuration) {
        /* stub */
    }

    @Override
    public void applyTurnTalonFXConfig(TalonFXConfiguration configuration) {
        /* stub */
    }

    @Override
    public void setDriveMotorVoltage(double newValue) {
        /* stub */
    }

    @Override
    public void setTurnMotorVoltage(double newValue) {
        /* stub */
    }

    @Override
    public void setTurnMotorMotionMagic(MotionMagicExpoVoltage request) {
        /* stub */
    }

    @Override
    public void setDriveMotorMotionMagic(MotionMagicVelocityVoltage request) {
        /* stub */
    }

    @Override
    public double getDriveMotorVoltage() {
        return 0;
    }

    @Override
    public double getTurnMotorVoltage() {
        return 0;
    }

    @Override
    public void setDriveMotorPercent(double newValue) {
        /* stub */
    }

    @Override
    public void setTurnMotorPercent(double newValue) {
        /* stub */
    }

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
    public double getTurnMechanismPosition() {
        return 0;
    }

    @Override
    public double getTurnMechanismVelocity() {
        return 0;
    }

    @Override
    public double getDriveMechanismPosition() {
        return 0;
    }

    @Override
    public double getDriveMechanismVelocity() {
        return 0;
    }

    @Override
    public double getDriveMechanismAcceleration() {
        return 0;
    }

    @Override
    public void setDriveMechanismPosition(double newValue) {
        /* stub */
    }

    @Override
    public void setTurnMechanismPosition(double newValue) {
        /* stub */
    }

    @Override
    public void close() throws Exception {
        /* stub */
    }

    @Override
    public double getDriveMotorCurrentDrawAmps() {
        return 0;
    }

    @Override
    public double getTurnMotorCurrentDrawAmps() {
        return 0;
    }

    @Override
    public void simulationPeriodic() {
        /* stub */
    }
}
