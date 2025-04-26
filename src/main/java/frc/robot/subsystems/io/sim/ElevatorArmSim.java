package frc.robot.subsystems.io.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.io.ElevatorArmIO;

public class ElevatorArmSim implements ElevatorArmIO {

  private final double periodicDt;

  private final TalonFX arm;

  private final TalonFXSimState armSim;

  private final DCMotor armMotor = DCMotor.getKrakenX60(1);

  private final SingleJointedArmSim armSimModel =
      new SingleJointedArmSim(
          LinearSystemId.createDCMotorSystem(Constants.ElevatorArm.kV, Constants.ElevatorArm.kA),
          armMotor,
          Constants.ElevatorArm.ARM_GEAR_RATIO,
          Units.inchesToMeters(14.4966),
          Units.degreesToRadians(-105),
          Units.degreesToRadians(115),
          true,
          Constants.ElevatorArm.START_POS,
          0.001,
          0.001);

  public ElevatorArmSim(double periodicDt) {
    this.periodicDt = periodicDt;

    arm = new TalonFX(RobotMap.ElevatorArm.ARM_ID);
    armSim = arm.getSimState();
    armSim.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void applyArmTalonFXConfig(TalonFXConfiguration configuration) {
    arm.getConfigurator().apply(configuration);
  }

  @Override
  public void setArmVoltage(double outputVolts) {
    arm.setVoltage(outputVolts);
  }

  @Override
  public void setArmPosition(double newValue) {
    arm.setPosition(newValue);
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
  public void setArmPercent(double newValue) {
    arm.set(newValue);
  }

  @Override
  public double getArmPercent() {
    return arm.get();
  }

  @Override
  public double getArmVoltage() {
    return arm.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public double getArmCurrentDrawAmps() {
    return arm.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void close() throws Exception {
    arm.close();
  }

  @Override
  public void simulationPeriodic() {
    armSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var armVoltage = armSim.getMotorVoltage();

    armSimModel.setInputVoltage(armVoltage);
    armSimModel.update(periodicDt);

    armSim.setRawRotorPosition(
        Constants.ElevatorArm.ARM_GEAR_RATIO
            * Units.radiansToRotations(
                armSimModel.getAngleRads() - Constants.ElevatorArm.START_POS));
    armSim.setRotorVelocity(
        Constants.ElevatorArm.ARM_GEAR_RATIO
            * Units.radiansToRotations(armSimModel.getVelocityRadPerSec()));
  }
}
