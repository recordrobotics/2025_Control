package frc.robot.subsystems.io.sim;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.NavSensorIO;

public class NavSensorSim implements NavSensorIO {

  private final AHRS _nav = new AHRS(NavXComType.kUSB1);

  @Override
  public void reset() {
    _nav.reset();
  }

  @Override
  public void resetDisplacement() {
    _nav.resetDisplacement();
  }

  @Override
  public double getAngle() {
    return _nav.getAngle();
  }

  @Override
  public double getWorldLinearAccelX() {
    return _nav.getWorldLinearAccelX();
  }

  @Override
  public double getWorldLinearAccelY() {
    return _nav.getWorldLinearAccelY();
  }

  @Override
  public boolean isConnected() {
    return _nav.isConnected();
  }

  @Override
  public void close() throws Exception {
    _nav.close();
  }

  private double angleRads = 0;

  @Override
  public void simulationPeriodic() {
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[2]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angleRads += RobotContainer.drivetrain.getChassisSpeeds().omegaRadiansPerSecond * 0.02;
    angle.set(Units.radiansToDegrees(angleRads));
  }
}
