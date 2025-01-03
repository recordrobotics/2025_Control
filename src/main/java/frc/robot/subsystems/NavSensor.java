package frc.robot.subsystems;

// AHRS Import
import com.kauailabs.navx.frc.AHRS;
// Other imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Smartdashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavSensor extends SubsystemBase {

	private static AHRS _nav = new AHRS(SerialPort.Port.kUSB1);

	private static double referenceAngle; // variable to keep track of a reference angle whenever you reset

	public static void initNav() {
		_nav.reset();
		_nav.resetDisplacement();
		referenceAngle = _nav.getAngle();
	}

	// Instance (I think I know what that means)
	public NavSensor() {
	}

	// stores the reference angle as whatever the angle is currently measured to be
	public void relativeResetAngle() {
		referenceAngle = _nav.getAngle();
	}

	// Gets the angle minus the reference angle
	public Rotation2d getAdjustedAngle() {
		return new Rotation2d(-(_nav.getAngle() - referenceAngle) / 180 * Math.PI);
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Nav connected", _nav.isConnected());
		// SmartDashboard.putBoolean("Nav Cal", _nav.isCalibrating());
		SmartDashboard.putNumber("getAngle()", _nav.getAngle());
	}

	/*
	 * public double getPitch() {
	 * double pitch = _nav.getRoll();
	 * return Units.degreesToRadians(pitch);
	 * }
	 * 
	 * /*
	 * public double getPitch() {
	 * double pitch = _nav.getRoll();
	 * return Units.degreesToRadians(pitch);
	 * }
	 * 
	 * public double getRoll() {
	 * double roll = _nav.getPitch();
	 * return Units.degreesToRadians(-1*roll);
	 * }
	 * 
	 * public double getYaw() {
	 * double yaw = _nav.getYaw();
	 * return Units.degreesToRadians(-1*yaw);
	 * }
	 * 
	 * //None of the below are guarenteed to work (weird axis changes)
	 * 
	 * public double getDisplacementX() {
	 * return _nav.getDisplacementX();
	 * }
	 * 
	 * public double getDisplacementY() {
	 * return _nav.getDisplacementY();
	 * }
	 * 
	 * public double getDisplacementZ() {
	 * return _nav.getDisplacementZ();
	 * }
	 * 
	 * void sensorResetAngle() {
	 * _nav.reset();
	 * }
	 * 
	 * void resetDisplacement() {
	 * _nav.resetDisplacement();
	 * }
	 * 
	 * void resetAll(){
	 * sensorResetAngle();
	 * resetDisplacement();
	 * 
	 * }
	 */
}