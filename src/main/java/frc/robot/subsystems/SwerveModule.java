package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.ModuleConstants;
import frc.robot.shuffleboard.ShuffleboardUI;

public class SwerveModule {

  // Creates variables for motors and absolute encoders
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final DutyCycleEncoder absoluteTurningMotorEncoder;
  private final double turningEncoderOffset;

  private final ProfiledPIDController drivePIDController;
  private final ProfiledPIDController turningPIDController;
  private final SimpleMotorFeedforward driveFeedForward;

  private final double TURN_GEAR_RATIO;
  private final double DRIVE_GEAR_RATIO;
  private final double WHEEL_DIAMETER;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, and absolute
   * turning encoder.
   * 
   * @param m - a ModuleConstants object that contains all constants relevant for
   *          creating a swerve module.
   *          Look at ModuleConstants.java for what variables are contained
   */
  public SwerveModule(ModuleConstants m) {


    // Creates TalonFX objects
    m_driveMotor = new TalonFX(m.driveMotorChannel);
    m_turningMotor = new TalonFX(m.turningMotorChannel);

    // Creates Motor Encoder object and gets offset
    absoluteTurningMotorEncoder = new DutyCycleEncoder(m.absoluteTurningMotorEncoderChannel);
    turningEncoderOffset = m.turningEncoderOffset;

    // Creates other variables
    this.TURN_GEAR_RATIO = m.TURN_GEAR_RATIO;
    this.DRIVE_GEAR_RATIO = m.DRIVE_GEAR_RATIO;
    this.WHEEL_DIAMETER = m.WHEEL_DIAMETER;

    // ~2 Seconds delay per swerve module
    Timer.delay(2.3);

    // Sets motor speeds to 0
    m_driveMotor.set(0);
    m_turningMotor.set(0);

    // Creates PID Controllers
    this.drivePIDController = new ProfiledPIDController(
        m.DRIVE_KP,
        m.DRIVE_KI,
        m.DRIVE_KD,
        new TrapezoidProfile.Constraints(m.DriveMaxAngularVelocity, m.DriveMaxAngularAcceleration));

    this.driveFeedForward = new SimpleMotorFeedforward(
        m.DRIVE_FEEDFORWARD_KS,
        m.DRIVE_FEEDFORWARD_KV);

    this.turningPIDController = new ProfiledPIDController(
        m.TURN_KP,
        m.TURN_KI,
        m.TURN_KD,
        new TrapezoidProfile.Constraints(m.TurnMaxAngularVelocity, m.TurnMaxAngularAcceleration));

    // Limit the PID Controller's input range between -0.5 and 0.5 and set the input to be continuous.
    turningPIDController.enableContinuousInput(-0.5, 0.5);

    // Corrects for offset in absolute motor position
    m_turningMotor.setPosition(getAbsWheelTurnOffset());
    
    // Sets up shuffleboard
    setupShuffleboard(m.driveMotorChannel, m.turningMotorChannel);
}

  /**
   * *custom function
   * 
   * @return The current offset absolute position of the wheel's turn
   */
  private double getAbsWheelTurnOffset() {
    double absEncoderPosition = (absoluteTurningMotorEncoder.getAbsolutePosition() - turningEncoderOffset + 1) % 1;
    double absWheelPositionOffset = absEncoderPosition * TURN_GEAR_RATIO;
    return absWheelPositionOffset;
  }

  /**
   * *custom function
   * 
   * @return The raw rotations of the turning motor (rotation 2d object).
   */
  private Rotation2d getTurnWheelRotation2d() {
    double numMotorRotations = m_turningMotor.getPosition().getValueAsDouble();
    Rotation2d motorRotation = new Rotation2d(numMotorRotations * 2 * Math.PI / TURN_GEAR_RATIO);
    return motorRotation;
  }

  /**
   * TODO: figure out how this calculation works and make it more clear instead of
   * having it all happen on one line
   * *custom function
   * 
   * @return The current velocity of the drive motor (meters per second)
   */
  private double getDriveWheelVelocity() {
    double driveMotorRotationsPerSecond = m_driveMotor.getVelocity().getValueAsDouble();
    double driveWheelMetersPerSecond = (driveMotorRotationsPerSecond / DRIVE_GEAR_RATIO)
        * (WHEEL_DIAMETER * Math.PI);
    return driveWheelMetersPerSecond;
  }

  /**
   * *custom function
   * 
   * @return The distance driven by the drive wheel (meters)
   */
  private double getDriveWheelDistance() {
    double numRotationsDriveMotor = m_driveMotor.getPosition().getValueAsDouble(); 
    double numRotationsDriveWheel = numRotationsDriveMotor / DRIVE_GEAR_RATIO;
    double speedWheelDistanceMeters = numRotationsDriveWheel * Math.PI * WHEEL_DIAMETER;
    return speedWheelDistanceMeters;
  }

  /**
   * *custom function
   * 
   * @return The current state of the module.
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
        getDriveWheelVelocity(), getTurnWheelRotation2d());
  }

  /**
   * *custom function
   * 
   * @return The current position of the module as a SwerveModulePosition object.
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        getDriveWheelDistance(), getTurnWheelRotation2d());
  }

  /** resets drive motor position */
  public void resetDriveMotorPosition() {
    m_driveMotor.setPosition(0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState,
        getTurnWheelRotation2d());

    // Calculate the drive output from the drive PID controller then set drive
    // motor.
    double driveOutput = drivePIDController.calculate(getDriveWheelVelocity(),
        optimizedState.speedMetersPerSecond);
    double driveFeedforwardOutput = driveFeedForward.calculate(optimizedState.speedMetersPerSecond);
    m_driveMotor.setVoltage(driveOutput + driveFeedforwardOutput);

    ShuffleboardUI.Autonomous.putSwerveVelocityData(m_driveMotor.getDeviceID(), getDriveWheelVelocity(), optimizedState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller then set
    // turn motor.
    final double turnOutput = turningPIDController.calculate(getTurnWheelRotation2d().getRotations(),
        optimizedState.angle.getRotations());
    m_turningMotor.set(turnOutput);
  }

  public void stop() {
    m_driveMotor.setVoltage(0);
    m_turningMotor.set(0);
  }

  // SHUFFLEBOARD STUFF

  private void setupShuffleboard(double driveMotorChannel, double turningMotorChannel) {
    ShuffleboardUI.Test.addMotor("Drive " + driveMotorChannel, m_driveMotor);
    ShuffleboardUI.Test.addMotor("Turn " + turningMotorChannel, m_turningMotor);
    ShuffleboardUI.Test.addNumber("Encoder " + absoluteTurningMotorEncoder.getSourceChannel(), absoluteTurningMotorEncoder::getAbsolutePosition);
  }
}