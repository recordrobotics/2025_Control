package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ElevatorIO;

public class ElevatorSim implements ElevatorIO {

  private final double periodicDt;

  private final TalonFX motorLeft;
  private final TalonFX motorRight;

  private final TalonFXSimState motorLeftSim;
  private final TalonFXSimState motorRightSim;

  private final edu.wpi.first.wpilibj.simulation.ElevatorSim elevatorSim =
      new edu.wpi.first.wpilibj.simulation.ElevatorSim(
          DCMotor.getKrakenX60(2),
          Constants.Elevator.GEAR_RATIO,
          1.0,
          Constants.Elevator.DRUM_RADIUS,
          0,
          Constants.Elevator.MAX_HEIGHT,
          true,
          0,
          0.0003,
          0.0003);

  private final DigitalInput bottomEndStop = new DigitalInput(RobotMap.Elevator.BOTTOM_ENDSTOP_ID);
  private final DigitalInput topEndStop = new DigitalInput(RobotMap.Elevator.TOP_ENDSTOP_ID);
  private final SimDevice bottomEndStopSim =
      SimDevice.create("DigitalInput", RobotMap.Elevator.BOTTOM_ENDSTOP_ID);
  private final SimDevice topEndStopSim =
      SimDevice.create("DigitalInput", RobotMap.Elevator.TOP_ENDSTOP_ID);
  private final SimBoolean bottomEndStopSimValue;
  private final SimBoolean topEndStopSimValue;

  public ElevatorSim(double periodicDt) {
    this.periodicDt = periodicDt;

    motorLeft = new TalonFX(RobotMap.Elevator.MOTOR_LEFT_ID);
    motorRight = new TalonFX(RobotMap.Elevator.MOTOR_RIGHT_ID);

    motorLeftSim = motorLeft.getSimState();
    motorRightSim = motorRight.getSimState();

    motorLeftSim.Orientation = ChassisReference.Clockwise_Positive;
    motorRightSim.Orientation = ChassisReference.Clockwise_Positive;

    if (bottomEndStopSim != null)
      bottomEndStopSimValue = bottomEndStopSim.createBoolean("Value", Direction.kOutput, false);
    else bottomEndStopSimValue = null;

    if (topEndStopSim != null)
      topEndStopSimValue = topEndStopSim.createBoolean("Value", Direction.kOutput, false);
    else topEndStopSimValue = null;

    if (bottomEndStopSim != null) bottomEndStop.setSimDevice(bottomEndStopSim);
    else bottomEndStop.close();

    if (topEndStopSim != null) topEndStop.setSimDevice(topEndStopSim);
    else topEndStop.close();
  }

  @Override
  public void applyTalonFXConfig(TalonFXConfiguration configuration) {
    motorLeft.getConfigurator().apply(configuration);
    motorRight.getConfigurator().apply(configuration);
  }

  @Override
  public void setLeftMotorVoltage(double outputVolts) {
    motorLeft.setVoltage(outputVolts);
  }

  @Override
  public void setRightMotorVoltage(double outputVolts) {
    motorRight.setVoltage(outputVolts);
  }

  @Override
  public double getLeftMotorVoltage() {
    return motorLeft.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public double getRightMotorVoltage() {
    return motorRight.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setLeftMotorPosition(double newValue) {
    motorLeft.setPosition(newValue);
  }

  @Override
  public void setRightMotorPosition(double newValue) {
    motorRight.setPosition(newValue);
  }

  @Override
  public double getLeftMotorPosition() {
    return motorLeft.getPosition().getValueAsDouble();
  }

  @Override
  public double getLeftMotorVelocity() {
    return motorLeft.getVelocity().getValueAsDouble();
  }

  @Override
  public double getRightMotorPosition() {
    return motorRight.getPosition().getValueAsDouble();
  }

  @Override
  public double getRightMotorVelocity() {
    return motorRight.getVelocity().getValueAsDouble();
  }

  @Override
  public void setLeftMotorPercent(double newValue) {
    motorLeft.set(newValue);
  }

  @Override
  public void setRightMotorPercent(double newValue) {
    motorRight.set(newValue);
  }

  @Override
  public double getLeftMotorPercent() {
    return motorLeft.get();
  }

  @Override
  public double getRightMotorPercent() {
    return motorRight.get();
  }

  @Override
  public boolean getTopEndStop() {
    if (topEndStopSimValue != null) return topEndStopSimValue.get();
    else return false;
    // return topEndStop.get();
  }

  @Override
  public boolean getBottomEndStop() {
    if (bottomEndStopSimValue != null) return bottomEndStopSimValue.get();
    else return false;
    // return bottomEndStop.get();
  }

  public void setTopEndStopSim(boolean newValue) {
    if (topEndStopSimValue != null) topEndStopSimValue.set(newValue);
  }

  public void setBottomEndStopSim(boolean newValue) {
    if (bottomEndStopSimValue != null) bottomEndStopSimValue.set(newValue);
  }

  @Override
  public double getLeftMotorCurrentDraw() {
    return motorLeft.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public double getRightMotorCurrentDraw() {
    return motorRight.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    motorLeft.close();
    motorRight.close();

    if (bottomEndStopSim != null) {
      bottomEndStopSim.close();
      bottomEndStop.close();
    }
    if (topEndStopSim != null) {
      topEndStopSim.close();
      topEndStop.close();
    }
  }

  @Override
  public void simulationPeriodic() {
    motorLeftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorRightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorLeftVoltage = motorLeftSim.getMotorVoltage();
    var motorRightVoltage = motorRightSim.getMotorVoltage();

    elevatorSim.setInputVoltage((motorLeftVoltage + motorRightVoltage) / 2.0);
    elevatorSim.update(periodicDt);

    motorLeftSim.setRawRotorPosition(
        elevatorSim.getPositionMeters() / Constants.Elevator.METERS_PER_ROTATION);
    motorLeftSim.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond() / Constants.Elevator.METERS_PER_ROTATION);
    motorRightSim.setRawRotorPosition(
        elevatorSim.getPositionMeters() / Constants.Elevator.METERS_PER_ROTATION);
    motorRightSim.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond() / Constants.Elevator.METERS_PER_ROTATION);
  }
}
