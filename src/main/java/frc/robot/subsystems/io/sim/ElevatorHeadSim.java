package frc.robot.subsystems.io.sim;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ElevatorHeadIO;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class ElevatorHeadSim implements ElevatorHeadIO {

  private final double periodicDt;

  private final SparkMax motor;
  private final SparkMaxSim motorSim;

  private boolean hasAlgae = false;

  private final DCMotor wheelMotor = DCMotor.getNeo550(1);

  private final DCMotorSim wheelSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(wheelMotor, 0.001, Constants.ElevatorHead.GEAR_RATIO),
          wheelMotor);

  private final DigitalInput coralDetector = new DigitalInput(RobotMap.ElevatorHead.PHOTOSENSOR_ID);
  private final SimDevice coralDetectorSim =
      SimDevice.create("DigitalInput", RobotMap.ElevatorHead.PHOTOSENSOR_ID);
  private final SimBoolean coralDetectorSimValue;

  private final AbstractDriveTrainSimulation drivetrainSim;

  public ElevatorHeadSim(double periodicDt, AbstractDriveTrainSimulation drivetrainSim) {
    this.periodicDt = periodicDt;
    this.drivetrainSim = drivetrainSim;

    motor = new SparkMax(RobotMap.ElevatorHead.MOTOR_ID, MotorType.kBrushless);
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

  public void setHasAlgae(boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
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

  public void setPreload() {
    setCoralDetectorSim(false);
    RobotContainer.model.getRobotCoral().poseSupplier =
        () -> RobotContainer.model.elevatorArm.getCoralShooterTargetPose();
  }

  @Override
  public void close() throws Exception {
    motor.close();
    if (coralDetectorSim != null) {
      coralDetectorSim.close();
      coralDetector.close();
    }
  }

  private MedianFilter velocityFilter = new MedianFilter(10);

  @Override
  public void simulationPeriodic() {
    var voltage = motorSim.getAppliedOutput() * motorSim.getBusVoltage();

    if (hasAlgae) {
      voltage /= 20.0;
    }

    wheelSimModel.setInputVoltage(voltage);
    wheelSimModel.update(periodicDt);

    motorSim.iterate(
        Units.radiansToRotations(wheelSimModel.getAngularVelocityRadPerSec())
            * 60.0
            * Constants.ElevatorHead.GEAR_RATIO,
        RobotController.getBatteryVoltage(),
        periodicDt);

    velocityFilter.calculate(RobotContainer.elevatorHead.getVelocity());

    if (!getCoralDetector()) { // NC
      var ejectPose =
          RobotContainer.model
              .elevatorArm
              .getCoralShooterTargetPose()
              .relativeTo(new Pose3d(RobotContainer.model.getRobot()));
      if (Math.abs(velocityFilter.lastValue()) > 1.3) {
        RobotContainer.model.getRobotCoral().poseSupplier = () -> null;
        setCoralDetectorSim(true); // NC

        var angle = ejectPose.getRotation().getMeasureY();
        if (RobotContainer.elevatorHead.getVelocity() > 0) {
          angle = angle.unaryMinus();
        }

        SimulatedArena.getInstance()
            .addGamePieceProjectile(
                new ReefscapeCoralOnFly(
                    // Obtain robot position from drive simulation
                    drivetrainSim.getSimulatedDriveTrainPose().getTranslation(),
                    ejectPose.toPose2d().getTranslation(),
                    // Obtain robot speed from drive simulation
                    drivetrainSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    // Obtain robot facing from drive simulation
                    drivetrainSim.getSimulatedDriveTrainPose().getRotation(),
                    // The height at which the coral is ejected
                    ejectPose.getMeasureZ(),
                    // The initial speed of the coral
                    MetersPerSecond.of(Math.abs(RobotContainer.elevatorHead.getVelocity())),
                    angle));
      }
    }
  }
}
