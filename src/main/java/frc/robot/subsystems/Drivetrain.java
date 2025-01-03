package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.DriveCommandData;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends KillableSubsystem implements ShuffleboardPublisher {
  // Creates swerve module objects
  private final SwerveModule m_frontLeft = new SwerveModule(Constants.Swerve.frontLeftConstants);
  private final SwerveModule m_frontRight = new SwerveModule(Constants.Swerve.frontRightConstants);
  private final SwerveModule m_backLeft = new SwerveModule(Constants.Swerve.backLeftConstants);
  private final SwerveModule m_backRight = new SwerveModule(Constants.Swerve.backRightConstants);

  // Creates swerve kinematics
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          Constants.Swerve.frontLeftConstants.wheelLocation,
          Constants.Swerve.frontRightConstants.wheelLocation,
          Constants.Swerve.backLeftConstants.wheelLocation,
          Constants.Swerve.backRightConstants.wheelLocation);

  public Drivetrain() {}

  /**
   * Drives the robot using joystick info.
   *
   * @param driveCommandData contains all the info to drive the robot. The xSpeed and ySpeed
   *     components are relative to the robot's current orientation if fieldRelative is false, and
   *     relative to the field if fieldRelative is true.
   * @param currentRotation the current rotation of the robot in the field coordinate system. Used
   *     to convert the joystick x and y components to the field coordinate system if fieldRelative
   *     is true.
   */
  public void drive(DriveCommandData driveCommandData) {
    // Data from driveCommandData
    boolean fieldRelative = driveCommandData.fieldRelative;
    double xSpeed = driveCommandData.xSpeed;
    double ySpeed = driveCommandData.ySpeed;
    double rot = driveCommandData.rot;

    // Calculates swerveModuleStates given optimal ChassisSpeeds given by control
    // scheme

    SmartDashboard.putNumber("rotation", rot);

    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    rot,
                    RobotContainer.poseTracker.getEstimatedPosition().getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // Desaturates wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.robotMaxSpeed);

    // Sets state for each module
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the PID target to zero and immediately stops all swerve modules.
   *
   * <p>This method commands the drivetrain to stop by setting the drive speeds to zero, thus
   * ensuring that the robot comes to a halt. It also directly stops each swerve module by setting
   * their motor outputs to zero.
   */
  @Override
  public void kill() {
    drive(new DriveCommandData(0, 0, 0, false));
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  /**
   * Retrieves the current chassis speeds relative to the robot's orientation.
   *
   * <p>This method calculates the chassis speeds based on the current states of all four swerve
   * modules using the drivetrain's kinematics.
   *
   * @return The current relative chassis speeds as a ChassisSpeeds object.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        m_frontLeft.getModuleState(),
        m_frontRight.getModuleState(),
        m_backLeft.getModuleState(),
        m_backRight.getModuleState());
  }

  /**
   * Returns the swerve drive kinematics for this drivetrain.
   *
   * @return The SwerveDriveKinematics object associated with this drivetrain.
   */
  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getModulePosition(),
      m_frontRight.getModulePosition(),
      m_backLeft.getModulePosition(),
      m_backRight.getModulePosition()
    };
  }

  /** frees up all hardware allocations */
  public void close() {
    NavSensor.getInstance().close();
    m_backLeft.close();
    m_backRight.close();
    m_frontLeft.close();
    m_frontRight.close();
  }

  @Override
  public void setupShuffleboard() {
    SwerveModule[] modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
    for (SwerveModule module : modules) {
      module.setupShuffleboard();
    }
  }
}
