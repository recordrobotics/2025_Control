package frc.robot.subsystems.io.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.CoralShooterIO;

public class CoralShooterSim implements CoralShooterIO {

  private final double periodicDt;

  private final SparkMax motor;
  private final SparkMaxSim motorSim;

  private final DCMotor wheelMotor = DCMotor.getNeo550(1);

  private final DCMotorSim wheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(wheelMotor, 0.001, Constants.CoralShooter.GEAR_RATIO),
          wheelMotor);

  private final DigitalInput coralDetector = new DigitalInput(RobotMap.CoralShooter.PHOTOSENSOR_ID);
  private final SimDevice coralDetectorSim =
      SimDevice.create("DigitalInput", RobotMap.CoralShooter.PHOTOSENSOR_ID);
  private final SimBoolean coralDetectorSimValue;

  public CoralShooterSim(double periodicDt) {
    this.periodicDt = periodicDt;

    motor = new SparkMax(RobotMap.CoralShooter.MOTOR_ID, MotorType.kBrushless);
    motorSim = new SparkMaxSim(motor, wheelMotor);

    if (coralDetectorSim != null)
      coralDetectorSimValue = coralDetectorSim.createBoolean("Value", Direction.kOutput, true);
    else coralDetectorSimValue = null;

    if (coralDetectorSim != null) coralDetector.setSimDevice(coralDetectorSim);
    else coralDetector.close();
  }

  @Override
  public void setVoltage(double outputVolts) {
    motor.setVoltage(outputVolts);
  }

  @Override
  public void setPosition(double newValue) {
    motor.getEncoder().setPosition(newValue);
  }

  @Override
  public double getPosition() {
    return motor.getEncoder().getPosition();
  }

  @Override
  public double getVelocity() {
    return motor.getEncoder().getVelocity();
  }

  @Override
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  @Override
  public void setPercent(double newValue) {
    motor.set(newValue);
  }

  @Override
  public double getPercent() {
    return motor.get();
  }

  @Override
  public boolean getCoralDetector() {
    if (coralDetectorSim != null) return coralDetectorSimValue.get();
    else return false;
  }

  public void setCoralDetectorSim(boolean newValue) {
    if (coralDetectorSimValue != null) coralDetectorSimValue.set(newValue);
  }

  @Override
  public double getCurrentDrawAmps() {
    return wheelSimModel.getCurrentDrawAmps();
  }

  @Override
  public void close() throws Exception {
    motor.close();
    if (coralDetectorSim != null) {
      coralDetectorSim.close();
      coralDetector.close();
    }
  }

  @Override
  public void simulationPeriodic() {
    var voltage = motorSim.getAppliedOutput() * motorSim.getBusVoltage();

    wheelSimModel.setInputVoltage(voltage);
    wheelSimModel.update(periodicDt);

    motorSim.iterate(
        Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
            * 60.0
            * Constants.CoralShooter.GEAR_RATIO,
        RobotController.getBatteryVoltage(),
        periodicDt);
  }
}
