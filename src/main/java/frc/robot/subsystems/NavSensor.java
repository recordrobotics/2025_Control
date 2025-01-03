package frc.robot.subsystems;

// AHRS Import
import com.kauailabs.navx.frc.AHRS;
// Other imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class NavSensor extends SubsystemBase {

	// Creates AHRS _nav object
	private static AHRS _nav = new AHRS(SerialPort.Port.kUSB1);

	// variable to keep track of a reference angle whenever you reset
	private static double referenceAngle;

	public static void initNav() {
		_nav.reset();
		_nav.resetDisplacement();
		referenceAngle = _nav.getAngle();
	}

	// Navsensor Constructor, Create instance
	public NavSensor() {
	}

	// stores the reference angle as whatever the angle is currently measured to be
	public void resetAngleAdjustment() {
		referenceAngle = _nav.getAngle();
	}

	// Gets the angle minus the reference angle
	public Rotation2d getAdjustedAngle() {
		return Rotation2d.fromDegrees(_nav.getAngle() - referenceAngle);
	}

	@Override
	public void periodic() {
	}
}