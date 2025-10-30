package frc.robot.subsystems.io.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ElevatorIO;

public class ElevatorReal implements ElevatorIO {

    @SuppressWarnings("unused")
    private final double periodicDt;

    private final TalonFX motorLead;
    private final TalonFX motorFollower;
    private final TalonFX arm;
    private final DigitalInput bottomEndStop;
    private final DigitalInput topEndStop;

    public ElevatorReal(double periodicDt) {
        this.periodicDt = periodicDt;

        motorLead = new TalonFX(RobotMap.Elevator.MOTOR_LEAD_ID);
        motorFollower = new TalonFX(RobotMap.Elevator.MOTOR_FOLLOWER_ID);
        arm = new TalonFX(RobotMap.ElevatorArm.ARM_ID);
        bottomEndStop = new DigitalInput(RobotMap.Elevator.BOTTOM_ENDSTOP_ID);
        topEndStop = new DigitalInput(RobotMap.Elevator.TOP_ENDSTOP_ID);
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        motorLead.getConfigurator().apply(configuration);
        motorFollower.getConfigurator().apply(configuration);
    }

    @Override
    public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
        arm.getConfigurator().apply(configuration);
    }

    @Override
    public Follower createFollower() {
        return new Follower(RobotMap.Elevator.MOTOR_LEAD_ID, false);
    }

    @Override
    public void setLeadMotorVoltage(double outputVolts) {
        motorLead.setVoltage(outputVolts);
    }

    @Override
    public void setArmVoltage(double outputVolts) {
        arm.setVoltage(outputVolts);
    }

    @Override
    public void setLeadMotionMagic(MotionMagicExpoVoltage request) {
        motorLead.setControl(request);
    }

    @Override
    public void setFollowerMotionMagic(Follower request) {
        motorFollower.setControl(request);
    }

    @Override
    public void setArmMotionMagic(MotionMagicExpoVoltage request) {
        arm.setControl(request);
    }

    @Override
    public double getLeadMotorVoltage() {
        return motorLead.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getFollowerMotorVoltage() {
        return motorFollower.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public double getArmVoltage() {
        return arm.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setLeadMotorPosition(double newValue) {
        motorLead.setPosition(newValue);
    }

    @Override
    public void setFollowerMotorPosition(double newValue) {
        motorFollower.setPosition(newValue);
    }

    @Override
    public void setArmPosition(double newValue) {
        arm.setPosition(newValue);
    }

    @Override
    public double getLeadMotorPosition() {
        return motorLead.getPosition().getValueAsDouble();
    }

    @Override
    public double getLeadMotorVelocity() {
        return motorLead.getVelocity().getValueAsDouble();
    }

    @Override
    public double getFollowerMotorPosition() {
        return motorFollower.getPosition().getValueAsDouble();
    }

    @Override
    public double getFollowerMotorVelocity() {
        return motorFollower.getVelocity().getValueAsDouble();
    }

    @Override
    public double getArmPosition() {
        return arm.getPosition().getValueAsDouble();
    }

    @Override
    public double getArmVelocity() {
        return arm.getVelocity().getValueAsDouble();
    }

    @Override
    public void setLeadMotorPercent(double newValue) {
        motorLead.set(newValue);
    }

    @Override
    public void setArmPercent(double newValue) {
        arm.set(newValue);
    }

    @Override
    public double getLeadMotorPercent() {
        return motorLead.get();
    }

    @Override
    public double getArmPercent() {
        return arm.get();
    }

    @Override
    public boolean isTopEndStopPressed() {
        return topEndStop.get();
    }

    @Override
    public boolean isBottomEndStopPressed() {
        return bottomEndStop.get();
    }

    @Override
    public double getLeadMotorCurrentDraw() {
        return motorLead.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getFollowerMotorCurrentDraw() {
        return motorFollower.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getArmCurrentDrawAmps() {
        return arm.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void close() throws Exception {
        motorLead.close();
        motorFollower.close();
        arm.close();
        bottomEndStop.close();
        topEndStop.close();
    }

    @Override
    public void simulationPeriodic() {
        /* real */
    }
}
