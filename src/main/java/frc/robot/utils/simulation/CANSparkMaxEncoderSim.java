package frc.robot.utils.simulation;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class CANSparkMaxEncoderSim implements RelativeEncoder {
  private double velocity;
  private double position;

  public CANSparkMaxEncoderSim(double velocity, double position) {
    this.velocity = velocity;
    this.position = position;
  }

  @Override
  public double getPosition() {
    return position;
  }

  @Override
  public double getVelocity() {
    return velocity;
  }

  // Required to be a RelativeEncoder but we don't need it so...

  public REVLibError setPosition(double position) {
    throw new UnsupportedOperationException();
  }

  public REVLibError setPositionConversionFactor(double conversionFactor) {
    throw new UnsupportedOperationException();
  }

  public REVLibError setVelocityConversionFactor(double factor) {
    throw new UnsupportedOperationException();
  }

  public double getPositionConversionFactor() {
    throw new UnsupportedOperationException();
  }

  public double getVelocityConversionFactor() {
    throw new UnsupportedOperationException();
  }

  public REVLibError setAverageDepth(int depth) {
    throw new UnsupportedOperationException();
  }

  public int getAverageDepth() {
    throw new UnsupportedOperationException();
  }

  public REVLibError setMeasurementPeriod(int period_ms) {
    throw new UnsupportedOperationException();
  }

  public int getMeasurementPeriod() {
    throw new UnsupportedOperationException();
  }

  public int getCountsPerRevolution() {
    throw new UnsupportedOperationException();
  }

  public REVLibError setInverted(boolean isInverted) {
    throw new UnsupportedOperationException();
  }

  public boolean getInverted() {
    throw new UnsupportedOperationException();
  }
}
