package frc.robot.subsystems.io.stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.io.ClimberIO;

public class ClimberStub implements ClimberIO {

  @SuppressWarnings("unused")
  private final double periodicDt;

  public ClimberStub(double periodicDt) {
    this.periodicDt = periodicDt;
  }

  @Override
  public void applyTalonFXConfig(TalonFXConfiguration configuration) {}

  @Override
  public void setVoltage(Voltage outputVolts) {}

  @Override
  public void setPosition(double newValue) {}

  @Override
  public double getPosition() {
    return 0;
  }

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public void setPercent(double newValue) {}

  @Override
  public double getPercent() {
    return 0;
  }

  @Override
  public double getCurrentDrawAmps() {
    return 0;
  }

  @Override
  public void simulationPeriodic() {}

  @Override
  public void close() {}
}
