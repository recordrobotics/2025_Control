package frc.robot.subsystems.io;

public interface ElevatorAlgaeIO extends AutoCloseable {

  public void setWheelVoltage(double outputVolts);

  public void setWheelPosition(double newValue);

  public double getWheelPosition();

  public double getWheelVelocity();

  public double getWheelVoltage();

  public void setWheelPercent(double newValue);

  public double getWheelPercent();

  public boolean getAlgaeDetector();

  public void simulationPeriodic();
}
