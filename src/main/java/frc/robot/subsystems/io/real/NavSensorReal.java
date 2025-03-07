package frc.robot.subsystems.io.real;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import frc.robot.subsystems.io.NavSensorIO;

public class NavSensorReal implements NavSensorIO {

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
    return -_nav.getAngle();
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

  @Override
  public void simulationPeriodic() {}
}
