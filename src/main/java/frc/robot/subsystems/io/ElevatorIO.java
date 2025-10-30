package frc.robot.subsystems.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public interface ElevatorIO extends AutoCloseable {

    void applyTalonFXConfig(TalonFXConfiguration configuration);

    void applyArmTalonFXConfig(TalonFXConfiguration configuration);

    Follower createFollower();

    void setLeadMotorVoltage(double outputVolts);

    void setArmVoltage(double outputVolts);

    double getLeadMotorVoltage();

    double getFollowerMotorVoltage();

    double getArmVoltage();

    void setLeadMotorPosition(double newValue);

    void setFollowerMotorPosition(double newValue);

    void setArmPosition(double newValue);

    void setLeadMotionMagic(MotionMagicExpoVoltage request);

    void setFollowerMotionMagic(Follower request);

    void setArmMotionMagic(MotionMagicExpoVoltage request);

    double getLeadMotorPosition();

    double getLeadMotorVelocity();

    double getFollowerMotorPosition();

    double getFollowerMotorVelocity();

    double getArmPosition();

    double getArmVelocity();

    void setLeadMotorPercent(double newValue);

    void setArmPercent(double newValue);

    double getLeadMotorPercent();

    double getArmPercent();

    boolean isTopEndStopPressed();

    boolean isBottomEndStopPressed();

    double getLeadMotorCurrentDraw();

    double getFollowerMotorCurrentDraw();

    double getArmCurrentDrawAmps();

    void simulationPeriodic();
}
