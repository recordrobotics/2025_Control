package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shuffleboard.ShuffleboardUI;

public class NavSensor extends SubsystemBase {

	private static double period = 0.02;

	/**
	 * The magnitude of a derivative of a vector
	 * is not equal to the derivative of a magnitude of a vector...
	 * because
	 * If a vector is on a unit circle, its magnitude is always 1
	 * however, that vector might change its angle and that is still
	 * a change in the vector that we need
	 * so we take the derivative first and then the magnitude.
	 */
	private double last_accelX;
	private double last_accelY;

	private double jerkX;
	private double jerkY;

	//Ceates AHRS _nav object
	private static AHRS _nav = new AHRS(SerialPort.Port.kUSB1);

	// variable to keep track of a reference angle whenever you reset
	private static double referenceAngle = _nav.getAngle();

	// Resets nav
	static {
		_nav.reset();
		_nav.resetDisplacement(); // Technically not necessary but whatever
		ShuffleboardUI.Overview.setNavSensor(_nav::isConnected);
		ShuffleboardUI.Test.addBoolean("Nav Sensor", _nav::isConnected);
	}

	// stores the reference angle as whatever the angle is currently measured to be
	public void resetAngleAdjustment() {
		referenceAngle = _nav.getAngle();
	}

	// Gets the angle minus the reference angle
	public Rotation2d getAdjustedAngle() {
		return Rotation2d.fromDegrees(-(_nav.getAngle() - referenceAngle));
	}

	public double getJerkMagnitude() {
		return Math.sqrt(jerkX * jerkX + jerkY * jerkY);
	}

	@Override
	public void periodic() {
		double accelX = _nav.getWorldLinearAccelX();
		double accelY = _nav.getWorldLinearAccelY(); // TODO: might need to be Z, should test
		jerkX = (accelX - last_accelX) / period;
		jerkY = (accelY - last_accelY) / period;
		last_accelX = accelX;
		last_accelY = accelY;
	}
}