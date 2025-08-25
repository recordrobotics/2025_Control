package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import frc.robot.subsystems.io.CoralIntakeIO;

public class CoralIntakeStub implements CoralIntakeIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    public CoralIntakeStub(double periodicDt) {
        this.periodicDt = periodicDt;
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        /* stub */
    }

    @Override
    public void setWheelVoltage(double outputVolts) {
        /* stub */
    }

    @Override
    public void setArmVoltage(double outputVolts) {
        /* stub */
    }

    @Override
    public void setWheelPosition(double newValue) {
        /* stub */
    }

    @Override
    public void setArmPosition(double newValue) {
        /* stub */
    }

    @Override
    public void setArmMotionMagic(MotionMagicExpoVoltage request) {
        /* stub */
    }

    @Override
    public double getWheelVoltage() {
        return 0;
    }

    @Override
    public double getArmVoltage() {
        return 0;
    }

    @Override
    public double getWheelPosition() {
        return 0;
    }

    @Override
    public double getWheelVelocity() {
        return 0;
    }

    @Override
    public double getArmPosition() {
        return 0;
    }

    @Override
    public double getArmVelocity() {
        return 0;
    }

    @Override
    public void setWheelPercent(double newValue) {
        /* stub */
    }

    @Override
    public void setArmPercent(double newValue) {
        /* stub */
    }

    @Override
    public double getWheelPercent() {
        return 0;
    }

    @Override
    public double getArmPercent() {
        return 0;
    }

    @Override
    public double getWheelCurrentDrawAmps() {
        return 0;
    }

    @Override
    public double getArmCurrentDrawAmps() {
        return 0;
    }

    @Override
    public void close() throws Exception {
        /* stub */
    }

    @Override
    public void simulationPeriodic() {
        /* stub */
    }
}
