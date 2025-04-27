package frc.robot.subsystems.io.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.io.NavSensorIO;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class NavSensorSim implements NavSensorIO {

  private final GyroSimulation gyroSimulation;

  public NavSensorSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void reset() {
    gyroSimulation.setRotation(new Rotation2d());
  }

  @Override
  public void resetDisplacement() {}

  @Override
  public double getAngle() {
    return gyroSimulation.getGyroReading().getDegrees();
  }

  @Override
  public double getWorldLinearAccelX() {
    return 0;
  }

  @Override
  public double getWorldLinearAccelY() {
    return 0;
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public void close() throws Exception {}

  // private double angleRads = 0;

  @Override
  public void simulationPeriodic() {
    // int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[2]");
    // SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // angleRads += RobotContainer.drivetrain.getChassisSpeeds().omegaRadiansPerSecond * 0.02;
    // angle.set(Units.radiansToDegrees(angleRads));
  }
}
