package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import frc.robot.subsystems.io.ElevatorIO;

public class ElevatorStub implements ElevatorIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  public ElevatorStub(double periodicDt) {
    this.periodicDt = periodicDt;
  }

  @Override
  public StrictFollower createFollower() {
    return new StrictFollower(0);
  }

  @Override
  public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

  @Override
  public void setLeadMotorVoltage(double outputVolts) {}

  @Override
  public void setLeadMotionMagic(MotionMagicExpoVoltage request) {}

  @Override
  public void setFollowerMotionMagic(StrictFollower request) {}

  @Override
  public double getLeadMotorVoltage() {
    return 0;
  }

  @Override
  public double getFollowerMotorVoltage() {
    return 0;
  }

  @Override
  public void setLeadMotorPosition(double newValue) {}

  @Override
  public void setFollowerMotorPosition(double newValue) {}

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
  public void setLeadMotorPercent(double newValue) {}

  @Override
  public double getLeadMotorPercent() {
    return 0;
  }

  @Override
  public boolean getTopEndStop() {
    return false;
  }

  @Override
  public boolean getBottomEndStop() {
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
  public void close() throws Exception {}

  @Override
  public void simulationPeriodic() {}
}
