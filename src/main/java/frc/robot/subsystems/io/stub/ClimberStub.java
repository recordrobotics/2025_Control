package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.robot.subsystems.io.ClimberIO;

public class ClimberStub implements ClimberIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    public ClimberStub(double periodicDt) {
        this.periodicDt = periodicDt;
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        /* stub */
    }

    @Override
    public void setVoltage(double outputVolts) {
        /* stub */
    }

    @Override
    public void setPosition(double newValue) {
        /* stub */
    }

    @Override
    public void setMotionMagic(MotionMagicVoltage request) {
        /* stub */
    }

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public void setPercent(double newValue) {
        /* stub */
    }

    @Override
    public double getPercent() {
        return 0;
    }

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public double getCurrentDrawAmps() {
        return 0;
    }

    @Override
    public void setRatchet(double value) {
        /* stub */
    }

    @Override
    public void simulationPeriodic() {
        /* stub */
    }

    @Override
    public void close() {
        /* stub */
    }
}
