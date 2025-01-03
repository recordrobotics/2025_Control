package frc.robot.utils.simulation;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.RobotBase;

public class CANSparkMaxWrapper {
  public CANSparkMax motor;
  public CANSparkMaxSim simMotor;

  public CANSparkMaxWrapper(int deviceId, MotorType type) {
    if (RobotBase.isReal()) {
      motor = new CANSparkMax(deviceId, type);
    } else {
      simMotor = new CANSparkMaxSim();
    }
  }

  public void set(double newSpeed) {
    if (RobotBase.isReal()) {
      motor.set(newSpeed);
    } else {
      simMotor.set(newSpeed);
    }
  }

  public double get() {
    if (RobotBase.isReal()) {
      return motor.get();
    } else {
      return simMotor.get();
    }
  }

  public RelativeEncoder getEncoder() {
    if (RobotBase.isReal()) {
      return motor.getEncoder();
    } else {
      return simMotor.getEncoder();
    }
  }

  public void setVoltage(double voltage) {
    if (RobotBase.isReal()) {
      motor.setVoltage(voltage);
    } else {
      simMotor.setVoltage(voltage);
    }
  }

  public void close() {
    if (RobotBase.isReal()) {
      motor.close();
    }
  }
}
