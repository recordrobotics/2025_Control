package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;

public class SwerveModule implements ShuffleboardPublisher, AutoCloseable, PoweredSubsystem {

  // Creates variables for motors and absolute encoders
  private int driveMotorChannel;
  private int turningMotorChannel;
  private int encoderChannel;

  private final SwerveModuleIO io;

  private final double turningEncoderOffset;

  private double targetDriveVelocity;
  private double targetTurnPosition;

  private final double TURN_GEAR_RATIO;
  private final double DRIVE_GEAR_RATIO;
  private final double WHEEL_DIAMETER;

  final MotionMagicVoltage turnRequest;
  final MotionMagicVelocityVoltage driveRequest;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, and absolute turning encoder.
   *
   * @param m - a ModuleConstants object that contains all constants relevant for creating a swerve
   *     module. Look at ModuleConstants.java for what variables are contained
   */
  public SwerveModule(ModuleConstants m, SwerveModuleIO io) {
    this.io = io;

    // Creates TalonFX objects
    driveMotorChannel = m.driveMotorChannel;
    turningMotorChannel = m.turningMotorChannel;
    encoderChannel = m.absoluteTurningMotorEncoderChannel;

    // Creates Motor Encoder object and gets offset
    turningEncoderOffset = m.turningEncoderOffset;

    // Creates other variables
    this.TURN_GEAR_RATIO = m.TURN_GEAR_RATIO;
    this.DRIVE_GEAR_RATIO = m.DRIVE_GEAR_RATIO;
    this.WHEEL_DIAMETER = m.WHEEL_DIAMETER;

    var driveConfig = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs_drive = driveConfig.Slot0;
    slot0Configs_drive.kS = m.DRIVE_KS;
    slot0Configs_drive.kV = m.DRIVE_KV;
    slot0Configs_drive.kA = m.DRIVE_KA;
    slot0Configs_drive.kP = 0.1;
    slot0Configs_drive.kI = 0;
    slot0Configs_drive.kD = 0;
    driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

    // set Motion Magic settings
    var motionMagicConfigs_drive = driveConfig.MotionMagic;
    motionMagicConfigs_drive.MotionMagicAcceleration = 400;
    motionMagicConfigs_drive.MotionMagicJerk = 4000;

    io.applyDriveTalonFXConfig(
        driveConfig
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(m.driveMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(m.driveMotorStatorCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)));

    var turnConfig = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs_turn = turnConfig.Slot0;
    slot0Configs_turn.kS = m.TURN_KS;
    slot0Configs_turn.kV = m.TURN_KV;
    slot0Configs_turn.kA = m.TURN_KA;
    slot0Configs_turn.kP = 4;
    slot0Configs_turn.kI = 0;
    slot0Configs_turn.kD = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;

    // set Motion Magic settings
    var motionMagicConfigs_turn = turnConfig.MotionMagic;
    motionMagicConfigs_turn.MotionMagicCruiseVelocity = m.TurnMaxAngularVelocity;
    motionMagicConfigs_turn.MotionMagicAcceleration = m.TurnMaxAngularAcceleration;
    motionMagicConfigs_turn.MotionMagicJerk = 1600;

    io.applyTurnTalonFXConfig(
        turnConfig
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(m.turnMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(m.turnMotorStatorCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)));

    // ~2 Seconds delay per swerve module to wait for encoder values to stabilize
    Timer.delay(2.3);

    // Sets motor speeds to 0
    io.setDriveMotorVoltage(0);
    io.setTurnMotorVoltage(0);

    // Corrects for offset in absolute wheel position
    io.setTurnMechanismPosition(getAbsWheelTurnOffset());
    turnRequest = new MotionMagicVoltage(getAbsWheelTurnOffset());
    driveRequest = new MotionMagicVelocityVoltage(0);

    SmartDashboard.putNumber("SwerveTurn_" + turningMotorChannel, 0);
  }

  /**
   * *custom function
   *
   * @return The current offset absolute position of the wheel's turn
   */
  private double getAbsWheelTurnOffset() {
    double absEncoderPosition = (io.getAbsoluteEncoder() - turningEncoderOffset + 1) % 1;
    return absEncoderPosition;
  }

  /**
   * *custom function
   *
   * @return The raw rotations of the turning motor (rotation 2d object).
   */
  public Rotation2d getTurnWheelRotation2d() {
    // Get the wheel's current turn position in rotations
    double numWheelRotations = io.getTurnMechanismPosition();
    // Convert wheel rotations to radians
    double wheelRadians = numWheelRotations * 2 * Math.PI;
    // Create a Rotation2d object from the wheel's angle in radians
    Rotation2d wheelRotation = new Rotation2d(wheelRadians);

    return wheelRotation;
  }

  public double getTurnWheelVelocity() {
    // RPS
    double turnWheelRotationsPerSecond = io.getTurnMechanismVelocity();
    return turnWheelRotationsPerSecond;
  }

  // meters per second
  public double getDriveWheelVelocity() {
    // Get the drive motor velocity in rotations per second
    double driveWheelRotationsPerSecond = io.getDriveMechanismVelocity();

    // Calculate the distance the wheel travels per rotation (circumference)
    double wheelCircumference = WHEEL_DIAMETER * Math.PI;

    // Calculate wheel velocity in meters per second
    double driveWheelMetersPerSecond = driveWheelRotationsPerSecond * wheelCircumference;

    return driveWheelMetersPerSecond;
  }

  public double getDriveWheelDistance() {
    // Get the drive wheel's current position in rotations
    double numRotationsDriveWheel = io.getDriveMechanismPosition();

    // Calculate the wheel's circumference
    double wheelCircumference = Math.PI * WHEEL_DIAMETER;

    // Calculate the total distance traveled by the wheel in meters
    double driveWheelDistanceMeters = numRotationsDriveWheel * wheelCircumference;

    return driveWheelDistanceMeters;
  }

  /**
   * *custom function
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(getDriveWheelVelocity(), getTurnWheelRotation2d());
  }

  /**
   * *custom function
   *
   * @return The current position of the module as a SwerveModulePosition object.
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getDriveWheelDistance(), getTurnWheelRotation2d());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(getTurnWheelRotation2d());

    targetTurnPosition = desiredState.angle.getRotations();
    targetDriveVelocity = desiredState.speedMetersPerSecond;

    controllerPeriodic();
  }

  public void controllerPeriodic() {
    double actualTargetDriveVelocity =
        targetDriveVelocity
            * Math.cos(
                Units.rotationsToRadians(targetTurnPosition)
                    - getTurnWheelRotation2d().getRadians());

    io.setDriveMotorMotionMagic(driveRequest.withVelocity(actualTargetDriveVelocity));
    io.setTurnMotorMotionMagic(turnRequest.withPosition(targetTurnPosition));
  }

  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public void stop() {
    targetDriveVelocity = 0;
    io.setDriveMotorVoltage(0);
    io.setTurnMotorVoltage(0);
  }

  public void setDriveMotorVoltsSysIdOnly(double volts) {
    io.setDriveMotorVoltage(volts);
  }

  public double getDriveMotorVoltsSysIdOnly() {
    return io.getDriveMotorVoltage();
  }

  public void setTurnMotorVoltsSysIdOnly(double volts) {
    io.setTurnMotorVoltage(volts);
  }

  public double getTurnMotorVoltsSysIdOnly() {
    return io.getTurnMotorVoltage();
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Drive " + driveMotorChannel, io.getDriveMotorPercent(), -1, 1)
        .subscribe(io::setDriveMotorPercent);

    DashboardUI.Test.addSlider("Turn " + turningMotorChannel, io.getTurnMotorPercent(), -1, 1)
        .subscribe(io::setTurnMotorPercent);

    DashboardUI.Test.addNumber("Encoder " + encoderChannel, io::getAbsoluteEncoder);
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    io.close();
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getDriveMotorCurrentDrawAmps() + io.getTurnMotorCurrentDrawAmps();
  }
}
