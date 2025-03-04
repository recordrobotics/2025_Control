package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.Logger;

public class SwerveModule implements ShuffleboardPublisher, AutoCloseable, PoweredSubsystem {

  // Creates variables for motors and absolute encoders
  private int driveMotorChannel;
  private int turningMotorChannel;
  private int encoderChannel;

  private final SwerveModuleIO io;

  private final double turningEncoderOffset;

  // Maximum velocity and acceleration constraints
  private final TrapezoidProfile.Constraints constraints;
  private final TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // The plant holds a state-space model of our turn motor. This system has the following
  // properties:
  //
  // States: [position, velocity], in, meters and meters per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in meters.
  //
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  private final LinearSystem<N2, N1, N2> turnSystem;

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N1, N2> turnObserver;

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N1, N2>
      turnController; // Nominal time between loops. 0.020 for TimedRobot, but can be lower if using
  // notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N2, N1, N2> turnLoop;

  private final double turn_kS;

  private final PIDController drivePID;
  private final SimpleMotorFeedforward driveFF;

  private double targetDriveVelocity;

  private final Notifier m_notifier;

  private final double TURN_GEAR_RATIO;
  private final double DRIVE_GEAR_RATIO;
  private final double WHEEL_DIAMETER;

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

    io.applyDriveTalonFXConfig(
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(m.driveMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(m.driveMotorStatorCurrentLimit)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)));

    io.applyTurnTalonFXConfig(
        new TalonFXConfiguration()
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

    this.constraints =
        new TrapezoidProfile.Constraints(m.TurnMaxAngularVelocity, m.TurnMaxAngularAcceleration);
    this.m_profile = new TrapezoidProfile(constraints);

    this.turnSystem = LinearSystemId.identifyPositionSystem(m.TURN_KV, m.TURN_KA);
    this.turnObserver =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            turnSystem,
            VecBuilder.fill(
                m.TURN_STD_STATE_POSITION,
                m.TURN_STD_STATE_VELOCITY), // Standard deviation of the state (position, velocity)
            VecBuilder.fill(
                m.TURN_STD_ENCODER_POSITION,
                m.TURN_STD_ENCODER_VELOCITY), // Standard deviation of encoder measurements
            // (position, velocity)
            Constants.Swerve.kDt);
    this.turnController =
        new LinearQuadraticRegulator<>(
            turnSystem,
            VecBuilder.fill(
                m.TURN_REGULATOR_POSITION_ERROR_TOLERANCE,
                m.TURN_REGULATOR_VELOCITY_ERROR_TOLERANCE), // qelms. Positon, Velocity error
            // tolerance, in meters and meters per second. Decrease this to more heavily
            // penalize state excursion, or make the controller behave more aggressively.
            VecBuilder.fill(
                m.TURN_REGULATOR_CONTROL_EFFORT_TOLERANCE), // relms. Control effort (voltage)
            // tolerance. Decrease this to more heavily penalize control effort, or
            // make the controller less aggressive. 12 is a good starting point because
            // that is the (approximate) maximum voltage of a battery.
            Constants.Swerve
                .kDt); // Nominal time between loops. 0.020 for TimedRobot, but can be lower if
    // using notifiers.

    turnController.latencyCompensate(turnSystem, Constants.Swerve.kDt, 0.020015);

    this.turnLoop =
        new LinearSystemLoop<>(
            turnSystem, turnController, turnObserver, 12.0, Constants.Swerve.kDt);

    turn_kS = m.TURN_KS;

    driveFF = new SimpleMotorFeedforward(m.DRIVE_KS, m.DRIVE_KV, m.DRIVE_KA);
    drivePID = new PIDController(0.01, 0, 0);

    // Corrects for offset in absolute motor position
    io.setTurnMotorPosition(getAbsWheelTurnOffset());
    m_setpoint = new TrapezoidProfile.State(getAbsWheelTurnOffset() / TURN_GEAR_RATIO, 0);

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
    double wheelRadians = motorRadians / TURN_GEAR_RATIO;

    // Create a Rotation2d object from the wheel's angle in radians
    Rotation2d wheelRotation = new Rotation2d(wheelRadians);

    return wheelRotation;
  }

  public double updateTargetRotation(double target, double current) {
    // return MathUtil.inputModulus(target, -1, 1) + current;
    // return ((target + 0.5) % 1.0 + 1.0) % 1.0 - 0.5 + Math.floor(current + 0.5);
    return target - Math.round(target - current);
  }

  public double getTurnWheelVelocity() {
    // RPS
    double turnMotorRotationsPerSecond = io.getTurnMotorVelocity();
    return turnMotorRotationsPerSecond / TURN_GEAR_RATIO;
  }

  // meters per second
  public double getDriveWheelVelocity() {
    // Get the drive motor velocity in rotations per second
    double driveMotorRotationsPerSecond = io.getDriveMotorVelocity();

    // Calculate wheel rotations per second by adjusting for the gear ratio
    double driveWheelRotationsPerSecond = driveMotorRotationsPerSecond / DRIVE_GEAR_RATIO;

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
    double numRotationsDriveWheel = numRotationsDriveMotor / DRIVE_GEAR_RATIO;

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

    Logger.recordOutput("DesiredState_" + turningMotorChannel, desiredState.angle.getRotations());

    m_goal =
        new TrapezoidProfile.State(
            updateTargetRotation(
                desiredState.angle.getRotations(), getTurnWheelRotation2d().getRotations()),
            // desiredState.angle.getRotations(),
            0);

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
    double nextDriveVoltage =
        drivePID.calculate(getDriveWheelVelocity(), targetDriveVelocity)
            + driveFF.calculateWithVelocities(lastSpeed, targetDriveVelocity);
    lastSpeed = targetDriveVelocity;

    // io.setDriveMotorVoltage(nextDriveVoltage);

    // Get next setpoint from profile.
    Logger.recordOutput("setpoint_" + turningMotorChannel, m_setpoint.position);
    m_setpoint = m_profile.calculate(Constants.Swerve.kDt, m_setpoint, m_goal);
    Logger.recordOutput("setpoint_after_" + turningMotorChannel, m_setpoint.position);

    // Set setpoint of the linear system (position m, velocity m/s).
    turnLoop.setNextR(VecBuilder.fill(m_setpoint.position, m_setpoint.velocity));

    // Correct our Kalman filter's state vector estimate with encoder data.
    turnLoop.correct(
        VecBuilder.fill(getTurnWheelRotation2d().getRotations(), getTurnWheelVelocity()));

    turnLoop.predict(Constants.Swerve.kDt);

    double nextturnVoltage = turnLoop.getU(0) + turn_kS * Math.signum(m_setpoint.velocity);

    Logger.recordOutput("targetvoltage_" + turningMotorChannel, nextturnVoltage);

    // io.setTurnMotorVoltage(nextturnVoltage);

    Logger.recordOutput("GoalPos_" + turningMotorChannel, m_goal.position);

    Logger.recordOutput("TargetVel_" + turningMotorChannel, m_setpoint.velocity);
    Logger.recordOutput("TargetPos_" + turningMotorChannel, m_setpoint.position);
    Logger.recordOutput("CurrentVel_" + turningMotorChannel, getTurnWheelVelocity());
    Logger.recordOutput(
        "CurrentPos_" + turningMotorChannel, getTurnWheelRotation2d().getRotations());

    Logger.recordOutput("TargetVel_" + driveMotorChannel, targetDriveVelocity);
    Logger.recordOutput("CurrentVel_" + driveMotorChannel, getDriveWheelVelocity());
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
