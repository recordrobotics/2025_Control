package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import frc.robot.subsystems.io.ElevatorIO;

public class ElevatorStub implements ElevatorIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    public ElevatorStub(double periodicDt) {
        this.periodicDt = periodicDt;
    }

    @Override
    public Follower createFollower() {
        return new Follower(0, false);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        /* stub */
    }

    @Override
    public void setLeadMotorVoltage(double outputVolts) {
        /* stub */
    }

    @Override
    public void setLeadMotionMagic(MotionMagicExpoVoltage request) {
        /* stub */
    }

    @Override
    public void setFollowerMotionMagic(Follower request) {
        /* stub */
    }

    @Override
    public double getLeadMotorVoltage() {
        return 0;
    }

    @Override
    public double getFollowerMotorVoltage() {
        return 0;
    }

    @Override
    public void setLeadMotorPosition(double newValue) {
        /* stub */
    }

    @Override
    public void setFollowerMotorPosition(double newValue) {
        /* stub */
    }

    @Override
    public double getLeadMotorPosition() {
        return 0;
    }

    @Override
    public double getLeadMotorVelocity() {
        return 0;
    }

    @Override
    public double getFollowerMotorPosition() {
        return 0;
    }

    @Override
    public double getFollowerMotorVelocity() {
        return 0;
    }

    @Override
    public void setLeadMotorPercent(double newValue) {
        /* stub */
    }

    @Override
    public double getLeadMotorPercent() {
        return 0;
    }

    @Override
    public boolean isTopEndStopPressed() {
        return false;
    }

    @Override
    public boolean isBottomEndStopPressed() {
        return false;
    }

    @Override
    public double getLeadMotorCurrentDraw() {
        return 0;
    }

    @Override
    public double getFollowerMotorCurrentDraw() {
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
