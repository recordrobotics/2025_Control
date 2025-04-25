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
import edu.wpi.first.wpilibj.Notifier;
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

  private final Notifier m_notifier;

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
    slot0Configs_drive.kS = m.DRIVE_KS; // Add 0.25 V output to overcome static friction
    slot0Configs_drive.kV = m.DRIVE_KV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs_drive.kA = m.DRIVE_KA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs_drive.kP = 0.1; // A position error of 2.5 rotations results in 12 V output
    slot0Configs_drive.kI = 0; // no output for integrated error
    slot0Configs_drive.kD = 0; // A velocity error of 1 rps results in 0.1 V output
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
    var slot0Configs = turnConfig.Slot0;
    slot0Configs.kS = m.TURN_KS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = m.TURN_KV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = m.TURN_KA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;

    // set Motion Magic settings
    var motionMagicConfigs = turnConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        m.TurnMaxAngularVelocity; // 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        m.TurnMaxAngularAcceleration; // 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

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

    // Corrects for offset in absolute motor position
    io.setTurnMotorPosition(getAbsWheelTurnOffset() / TURN_GEAR_RATIO);
    turnRequest = new MotionMagicVoltage(getAbsWheelTurnOffset() / TURN_GEAR_RATIO);
    driveRequest = new MotionMagicVelocityVoltage(0);

    SmartDashboard.putNumber("SwerveTurn_" + turningMotorChannel, 0);

    m_notifier = new Notifier(this::controllerPeriodic);
    // m_notifier.startPeriodic(Constants.Swerve.kDt);
  }

  /**
   * *custom function
   *
   * @return The current offset absolute position of the wheel's turn
   */
  private double getAbsWheelTurnOffset() {
    double absEncoderPosition = (io.getAbsoluteEncoder() - turningEncoderOffset + 1) % 1;
    double absWheelPositionOffset = absEncoderPosition * TURN_GEAR_RATIO;
    return absWheelPositionOffset;
  }

  /**
   * *custom function
   *
   * @return The raw rotations of the turning motor (rotation 2d object).
   */
  public Rotation2d getTurnWheelRotation2d() {
    // Get the turning motor's current position in rotations
    double numMotorRotations = io.getTurnMotorPosition();

    // Convert motor rotations to radians
    double motorRadians = numMotorRotations * 2 * Math.PI;

    // Adjust for the gear ratio to get wheel radians
    double wheelRadians = motorRadians;

    // Create a Rotation2d object from the wheel's angle in radians
    Rotation2d wheelRotation = new Rotation2d(wheelRadians);

    return wheelRotation;
  }

  public double getTurnWheelVelocity() {
    // RPS
    double turnMotorRotationsPerSecond = io.getTurnMotorVelocity();
    return turnMotorRotationsPerSecond;
  }

  // meters per second
  public double getDriveWheelVelocity() {
    // Get the drive motor velocity in rotations per second
    double driveMotorRotationsPerSecond = io.getDriveMotorVelocity();

    // Calculate wheel rotations per second by adjusting for the gear ratio
    double driveWheelRotationsPerSecond = driveMotorRotationsPerSecond;

    // Calculate the distance the wheel travels per rotation (circumference)
    double wheelCircumference = WHEEL_DIAMETER * Math.PI;

    // Calculate wheel velocity in meters per second
    double driveWheelMetersPerSecond = driveWheelRotationsPerSecond * wheelCircumference;

    return driveWheelMetersPerSecond;
  }

  public double getDriveWheelDistance() {
    // Get the drive motor's current position in rotations
    double numRotationsDriveMotor = io.getDriveMotorPosition();

    // Adjust for the gear ratio to get the number of wheel rotations
    double numRotationsDriveWheel = numRotationsDriveMotor;

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

  double lastSpeed = 0;

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(getTurnWheelRotation2d());

    // Logger.recordOutput("DesiredState_" + turningMotorChannel,
    // desiredState.angle.getRotations());

    targetTurnPosition = desiredState.angle.getRotations();

    // m_goal =
    //     new TrapezoidProfile.State(
    //         SmartDashboard.getNumber("SwerveTurn_" + turningMotorChannel, 0), 0);

    targetDriveVelocity = desiredState.speedMetersPerSecond;

    // ShuffleboardUI.Autonomous.putSwerveVelocityData(
    //   m_driveMotor.getDeviceID(), getDriveWheelVelocity(), drivePIDController.getSetpoint());

    // ShuffleboardUI.Autonomous.putSwerveVelocityData(
    //     m_driveMotor.getDeviceID(),
    //     getTurnWheelRotation2d().getRotations(),
    //     desiredState.angle.getRotations());

    controllerPeriodic();
  }

  public void controllerPeriodic() {
    double actualTargetDriveVelocity =
        targetDriveVelocity
            * Math.cos(
                Units.rotationsToRadians(targetTurnPosition)
                    - getTurnWheelRotation2d().getRadians());

    // double nextDriveVoltage =
    //     drivePID.calculate(getDriveWheelVelocity(), actualTargetDriveVelocity)
    //         + driveFF.calculateWithVelocities(lastSpeed, actualTargetDriveVelocity);
    // lastSpeed = actualTargetDriveVelocity;

    io.setDriveMotorMotionMagic(driveRequest.withVelocity(actualTargetDriveVelocity));

    io.setTurnMotorMotionMagic(turnRequest.withPosition(targetTurnPosition));

    // // Get next setpoint from profile.
    // // Logger.recordOutput("setpoint_" + turningMotorChannel, m_setpoint.position);
    // m_setpoint = m_profile.calculate(Constants.Swerve.kDt, m_setpoint, m_goal);
    // // Logger.recordOutput("setpoint_after_" + turningMotorChannel, m_setpoint.position);

    // // Set setpoint of the linear system (position m, velocity m/s).
    // turnLoop.setNextR(VecBuilder.fill(m_setpoint.position, m_setpoint.velocity));

    // // Correct our Kalman filter's state vector estimate with encoder data.
    // turnLoop.correct(
    //     VecBuilder.fill(getTurnWheelRotation2d().getRotations(), getTurnWheelVelocity()));

    // turnLoop.predict(Constants.Swerve.kDt);

    // double nextturnVoltage = turnLoop.getU(0) + turn_kS * Math.signum(m_setpoint.velocity);

    // // Logger.recordOutput("targetvoltage_" + turningMotorChannel, nextturnVoltage);

    // io.setTurnMotorVoltage(nextturnVoltage);

    // Logger.recordOutput("GoalPos_" + turningMotorChannel, m_goal.position);

    // Logger.recordOutput("TargetVel_" + turningMotorChannel, m_setpoint.velocity);
    // Logger.recordOutput("TargetPos_" + turningMotorChannel, m_setpoint.position);
    // Logger.recordOutput("CurrentVel_" + turningMotorChannel, getTurnWheelVelocity());
    // Logger.recordOutput(
    //     "CurrentPos_" + turningMotorChannel, getTurnWheelRotation2d().getRotations());

    // Logger.recordOutput("TargetVel_" + driveMotorChannel, targetDriveVelocity);
    // Logger.recordOutput("CurrentVel_" + driveMotorChannel, getDriveWheelVelocity());
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
    ;
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
    m_notifier.close();
    io.close();
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getDriveMotorCurrentDrawAmps() + io.getTurnMotorCurrentDrawAmps();
  }
}
