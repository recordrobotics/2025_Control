package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ShuffleboardPublisher;

public class SwerveModule implements ShuffleboardPublisher, AutoCloseable {

  // Creates variables for motors and absolute encoders
  private int driveMotorChannel;
  private int turningMotorChannel;
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final DutyCycleEncoder absoluteTurningMotorEncoder;
  private final double turningEncoderOffset;

  private final PIDController drivePIDController;
  private final SimpleMotorFeedforward driveFeedForward;

  private final ProfiledPIDController turningPIDController;
  private final SimpleMotorFeedforward turnFeedForward;

  // Maximum elevator velocity and acceleration constraints
  // private final TrapezoidProfile.Constraints constraints;
  // private final TrapezoidProfile m_profile;
  // private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  // private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  // The plant holds a state-space model of our turn motor. This system has the following
  // properties:
  //
  // States: [position, velocity], in, meters and meters per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [position], in meters.
  //
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  // private final LinearSystem<N2, N1, N2> turnSystem;

  // // The observer fuses our encoder data and voltage inputs to reject noise.
  // private final KalmanFilter<N2, N1, N2> turnObserver;

  // // A LQR uses feedback to create voltage commands.
  // private final LinearQuadraticRegulator<N2, N1, N2>
  //     turnController; // Nominal time between loops. 0.020 for TimedRobot, but can be
  // // lower if using notifiers.

  // // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  // private final LinearSystemLoop<N2, N1, N2> turnLoop;

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
  public SwerveModule(ModuleConstants m) {

    // Creates TalonFX objects
    driveMotorChannel = m.driveMotorChannel;
    turningMotorChannel = m.turningMotorChannel;
    m_driveMotor = new TalonFX(m.driveMotorChannel);
    m_turningMotor = new TalonFX(m.turningMotorChannel);

    // Creates Motor Encoder object and gets offset
    absoluteTurningMotorEncoder = new DutyCycleEncoder(m.absoluteTurningMotorEncoderChannel);
    turningEncoderOffset = m.turningEncoderOffset;

    // Creates other variables
    this.TURN_GEAR_RATIO = m.TURN_GEAR_RATIO;
    this.DRIVE_GEAR_RATIO = m.DRIVE_GEAR_RATIO;
    this.WHEEL_DIAMETER = m.WHEEL_DIAMETER;

    // ~2 Seconds delay per swerve module to wait for encoder values to stabilize
    Timer.delay(2.3);

    // Sets motor speeds to 0
    m_driveMotor.set(0);
    m_turningMotor.set(0);

    // Creates PID Controllers
    this.drivePIDController = new PIDController(m.DRIVE_KP, m.DRIVE_KI, m.DRIVE_KD);

    this.driveFeedForward =
        new SimpleMotorFeedforward(
            m.DRIVE_FEEDFORWARD_KS, m.DRIVE_FEEDFORWARD_KV, m.DRIVE_FEEDFORWARD_KA);

    // this.constraints =
    //     new TrapezoidProfile.Constraints(m.TurnMaxAngularVelocity, m.TurnMaxAngularAcceleration);
    // this.m_profile = new TrapezoidProfile(constraints);

    // this.turnSystem = LinearSystemId.identifyPositionSystem(m.TURN_KV, m.TURN_KA);
    // this.turnObserver =
    //     new KalmanFilter<>(
    //         Nat.N2(),
    //         Nat.N2(),
    //         turnSystem,
    //         VecBuilder.fill(
    //             m.TURN_STD_STATE_POSITION,
    //             m.TURN_STD_STATE_VELOCITY), // Standard deviation of the state (position, velocity)
    //         VecBuilder.fill(
    //             m.TURN_STD_ENCODER_POSITION,
    //             m.TURN_STD_ENCODER_VELOCITY), // Standard deviation of encoder measurements
    //         // (position,
    //         // velocity)
    //         Constants.Swerve.kDt);
    // this.turnController =
    //     new LinearQuadraticRegulator<>(
    //         turnSystem,
    //         VecBuilder.fill(
    //             m.TURN_REGULATOR_POSITION_ERROR_TOLERANCE,
    //             m.TURN_REGULATOR_VELOCITY_ERROR_TOLERANCE), // qelms. Positon, Velocity error
    //         // tolerance,
    //         // in meters and meters per second.
    //         // Decrease
    //         // this to more heavily penalize state excursion, or make the controller behave more
    //         // aggressively.
    //         VecBuilder.fill(
    //             m.TURN_REGULATOR_CONTROL_EFFORT_TOLERANCE), // relms. Control effort (voltage)
    //         // tolerance. Decrease this to more
    //         // heavily penalize control effort, or make the controller less aggressive. 12 is a good
    //         // starting point because that is the (approximate) maximum voltage of a battery.
    //         Constants.Swerve.kDt); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // // lower if using notifiers.

    // turnController.latencyCompensate(turnSystem, Constants.Swerve.kDt, 0.019895);

    // this.turnLoop =
    //     new LinearSystemLoop<>(
    //         turnSystem, turnController, turnObserver, 12.0, Constants.Swerve.kDt);

    this.turningPIDController =
        new ProfiledPIDController(
            m.TURN_KP,
            m.TURN_KI,
            m.TURN_KD,
            new TrapezoidProfile.Constraints(
                m.TurnMaxAngularVelocity, m.TurnMaxAngularAcceleration), Constants.Swerve.kDt);

    this.turnFeedForward =
    new SimpleMotorFeedforward(
        m.TURN_FEEDFORWARD_KS, m.TURN_FEEDFORWARD_KV, m.TURN_FEEDFORWARD_KA);

    // Limit the PID Controller's input range between -0.5 and 0.5 and set the input to be
    // continuous.
    turningPIDController.enableContinuousInput(-0.5, 0.5);

    // Corrects for offset in absolute motor position
    m_turningMotor.setPosition(getAbsWheelTurnOffset());

    m_notifier = new Notifier(this::controllerPeriodic);
    //m_notifier.startPeriodic(Constants.Swerve.kDt);
  }

  /**
   * *custom function
   *
   * @return The current offset absolute position of the wheel's turn
   */
  private double getAbsWheelTurnOffset() {
    double absEncoderPosition = (absoluteTurningMotorEncoder.get() - turningEncoderOffset + 1) % 1;
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
    double numMotorRotations = m_turningMotor.getPosition().getValueAsDouble();

    // Convert motor rotations to radians
    double motorRadians = numMotorRotations * 2 * Math.PI;

    // Adjust for the gear ratio to get wheel radians
    double wheelRadians = motorRadians / TURN_GEAR_RATIO;

    // Create a Rotation2d object from the wheel's angle in radians
    Rotation2d wheelRotation = new Rotation2d(wheelRadians);

    return wheelRotation;
  }

  public double getTurnWheelVelocity() {
    // RPS
    double turnMotorRotationsPerSecond = m_turningMotor.getVelocity().getValueAsDouble();
    return turnMotorRotationsPerSecond / TURN_GEAR_RATIO;
  }

  // meters per second
  public double getDriveWheelVelocity() {
    // Get the drive motor velocity in rotations per second
    double driveMotorRotationsPerSecond = m_driveMotor.getVelocity().getValueAsDouble();

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
    double numRotationsDriveMotor = m_driveMotor.getPosition().getValueAsDouble();

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

  private double lastSpeedMetersPerSecond = 0;
  private double lastTurnRotationsPerSecond = 0;

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(getTurnWheelRotation2d());
    drivePIDController.setSetpoint(desiredState.speedMetersPerSecond);
    turningPIDController.setGoal(desiredState.angle.getRotations());

    // ShuffleboardUI.Autonomous.putSwerveVelocityData(
    //   m_driveMotor.getDeviceID(), getDriveWheelVelocity(), drivePIDController.getSetpoint());

    // ShuffleboardUI.Autonomous.putSwerveVelocityData(
    //     m_driveMotor.getDeviceID(),
    //     getTurnWheelRotation2d().getRotations(),
    //     desiredState.angle.getRotations());

    controllerPeriodic();
  }

  public void controllerPeriodic() {
    // Calculate the drive output from the drive PID controller then set drive
    // motor.
    double drivePIDOutput = drivePIDController.calculate(getDriveWheelVelocity());
    double driveFeedforwardOutput =
        driveFeedForward.calculateWithVelocities(
            lastSpeedMetersPerSecond, drivePIDController.getSetpoint());
    m_driveMotor.setVoltage(
        drivePIDOutput + driveFeedforwardOutput); // Feed forward runs on voltage control

    // Calculate the turning motor output from the turning PID controller then set
    // turn motor.

    // Get next setpoint from profile.
    // m_setpoint = m_profile.calculate(Constants.Swerve.kDt, m_setpoint, m_goal);

    // // Set setpoint of the linear system (position m, velocity m/s).
    // turnLoop.setNextR(VecBuilder.fill(m_setpoint.position, m_setpoint.velocity));

    // // Correct our Kalman filter's state vector estimate with encoder data.
    // turnLoop.correct(
    //     VecBuilder.fill(getTurnWheelRotation2d().getRotations(), getTurnWheelVelocity()));

    // // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // // state with out Kalman filter.
    // turnController.calculate(turnObserver.getXhat(), turnLoop.getNextR());
    // var error = turnController.getR().minus(turnObserver.getXhat());
    // error.set(0, 0, MathUtil.inputModulus(error.get(0, 0), -0.5, 0.5));
    // var u =
    //     turnController
    //         .getK()
    //         .times(error)
    //         .plus(turnLoop.getFeedforward().calculate(turnLoop.getNextR()));
    // turnObserver.predict(u, Constants.Swerve.kDt);

    // double nextVoltage = turnLoop.getU(0);

    double turnPid = turningPIDController.calculate(getTurnWheelRotation2d().getRotations());
    double turnFeedforwardOutput =
        turnFeedForward.calculateWithVelocities(
            lastTurnRotationsPerSecond, turningPIDController.getSetpoint().velocity);

    double nextVoltage = turnPid + turnFeedforwardOutput;

    m_turningMotor.setVoltage(nextVoltage);

    Logger.recordOutput("TargetVel_" + m_turningMotor.getDeviceID(), turningPIDController.getSetpoint().velocity);
    Logger.recordOutput("TargetPos_" + m_turningMotor.getDeviceID(), turningPIDController.getSetpoint().position);
    Logger.recordOutput("CurrentVel_" + m_turningMotor.getDeviceID(), getTurnWheelVelocity());
    Logger.recordOutput("CurrentPos_" + m_turningMotor.getDeviceID(), getTurnWheelRotation2d().getRotations());

    lastSpeedMetersPerSecond = drivePIDController.getSetpoint();
    lastTurnRotationsPerSecond = turningPIDController.getSetpoint().velocity;
  }

  public void stop() {
    drivePIDController.setSetpoint(0);
    m_driveMotor.setVoltage(0); // Feed forward runs on voltage control
    m_turningMotor.setVoltage(0);
  }

  public void setDriveMotorVoltsSysIdOnly(double volts) {
    m_driveMotor.setVoltage(volts);
  }

  public double getDriveMotorVoltsSysIdOnly() {
    return m_driveMotor.getMotorVoltage().getValueAsDouble();
  }

  public void setTurnMotorVoltsSysIdOnly(double volts) {
    m_turningMotor.setVoltage(volts);
  }

  public double getTurnMotorVoltsSysIdOnly() {
    return m_turningMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Drive " + driveMotorChannel, m_driveMotor.get(), -1, 1)
        .subscribe(m_driveMotor::set);

    DashboardUI.Test.addSlider("Turn " + turningMotorChannel, m_turningMotor.get(), -1, 1)
        .subscribe(m_turningMotor::set);

    DashboardUI.Test.addNumber(
        "Encoder " + absoluteTurningMotorEncoder.getSourceChannel(),
        absoluteTurningMotorEncoder::get);
  }

  /** frees up all hardware allocations */
  public void close() {
    m_notifier.close();
    m_driveMotor.close();
    m_turningMotor.close();
    absoluteTurningMotorEncoder.close();
  }
}
