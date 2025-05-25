package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.NavSensorIO;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.ShuffleboardPublisher;

public class NavSensor extends ManagedSubsystemBase implements ShuffleboardPublisher {

  private static double period = 0.02;

  private final NavSensorIO io;

  /**
   * The magnitude of a derivative of a vector is not equal to the derivative of a magnitude of a
   * vector... because If a vector is on a unit circle, its magnitude is always 1 however, that
   * vector might change its angle and that is still a change in the vector that we need so we take
   * the derivative first and then the magnitude.
   */
  private double last_accelX;

  private double last_accelY;

  private double jerkX;
  private double jerkY;

  // variable to keep track of a reference angle whenever you reset
  private double referenceAngle;

  private static NavSensor instance;

  public static NavSensor getInstance() {
    return instance;
  }

  public NavSensor(NavSensorIO io) {
    instance = this;
    this.io = io;

    io.reset();
    io.resetDisplacement(); // Technically not necessary but whatever

    referenceAngle = io.getAngle();
  }

  // Stores the reference angle as whatever the angle is currently measured to be
  public void resetAngleAdjustment() {
    referenceAngle = io.getAngle();
  }

  // Gets the angle minus the reference angle
  public Rotation2d getAdjustedAngle() {
    return Rotation2d.fromDegrees((io.getAngle() - referenceAngle));
  }

  public double getJerkMagnitude() {
    return Math.sqrt(jerkX * jerkX + jerkY * jerkY);
  }

  @Override
  public void periodicManaged() {
    double accelX = io.getWorldLinearAccelX();
    double accelY = io.getWorldLinearAccelY();
    jerkX = (accelX - last_accelX) / period;
    jerkY = (accelY - last_accelY) / period;
    last_accelX = accelX;
    last_accelY = accelY;
  }

  @Override
  public void simulationPeriodicManaged() {
    io.simulationPeriodic();
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    io.close();
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Overview.setNavSensor(io::isConnected);
    DashboardUI.Test.addBoolean("Nav Sensor", io::isConnected);
  }
}
