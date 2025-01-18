// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.utils.KillableSubsystem;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends KillableSubsystem {
  private final SparkMax motor;
  private final SparkMax servo;

  private final DigitalInput coralDetector = new DigitalInput(RobotMap.CoralIntake.LIMIT_SWITCH_ID);

  private final ProfiledPIDController servoPID =
      new ProfiledPIDController(
          Constants.CoralIntake.sP,
          Constants.CoralIntake.sI,
          Constants.CoralIntake.sD,
          new Constraints(
              Constants.CoralIntake.MAX_SERVO_VELOCITY,
              Constants.CoralIntake.MAX_SERVO_ACCELERATION));
  private final ArmFeedforward servoFeedForward =
      new ArmFeedforward(
          Constants.CoralIntake.sS,
          Constants.CoralIntake.sG,
          Constants.CoralIntake.sV,
          Constants.CoralIntake.sA); // Feedforward for servo

  private final PIDController pid =
      new PIDController(
          Constants.CoralIntake.kP, Constants.CoralIntake.kI, Constants.CoralIntake.kD);

  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(Constants.CoralIntake.kS, Constants.CoralIntake.kV);

  public CoralIntake() {
    motor = new SparkMax(RobotMap.CoralIntake.MOTOR_ID, MotorType.kBrushless);
    servo = new SparkMax(RobotMap.CoralIntake.SERVO_ID, MotorType.kBrushless);
    toggle(CoralIntakeStates.OFF); // initialize as off
    ShuffleboardUI.Test.addSlider("Coral Intake", motor.get(), -1, 1).subscribe(motor::set);
    ShuffleboardUI.Test.addSlider("Coral Intake Pos", servo.getEncoder().getPosition(), -1, 1)
        .subscribe(this::toggleServo);

    sysIdRoutineWheel =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state -> Logger.recordOutput("SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                motor::setVoltage,
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                  // Record a frame for the shooter motor.
                  log.motor("coral-intake-wheel")
                      .voltage(
                          m_appliedVoltage.mut_replace(
                              motor.get() * RobotController.getBatteryVoltage(), Volts))
                      .angularPosition(m_angle.mut_replace(getWheelPosition(), Rotations))
                      .angularVelocity(
                          m_velocity.mut_replace(getWheelVelocity(), RotationsPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state
                // in
                // WPILog with this subsystem's name 
                this));
  }

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine sysIdRoutineWheel;

  public enum CoralIntakeStates {
    REVERSE,
    INTAKE,
    OFF;
  }

  public boolean hasCoral() {
    return coralDetector.get();
  }

  public enum IntakeServoStates {
    UP,
    DOWN,
    OFF;
  }

  public double getWheelVelocity() {
    return motor.getEncoder().getVelocity() / 60.0; /* RPM -> RPS */
  }

  public double getWheelPosition() {
    return motor.getEncoder().getPosition() / 60.0; /* RPM -> RPS */
  }

  public double getServoAngle() {
    return servo.getEncoder().getPosition() * Math.PI * 2;
  }

  /** Set the current shooter speed on both wheels to speed */
  public void toggle(double speed) {
    pid.setSetpoint(speed);
  }

  public void toggleServo(double pos) {
    servoPID.setGoal(pos);
  }

  public boolean atGoal() {
    return servoPID.atGoal();
  }

  public void toggleServo(IntakeServoStates state) {
    switch (state) {
      case UP:
        toggleServo(Constants.CoralIntake.SERVO_UP);
        break;
      case DOWN:
        toggleServo(Constants.CoralIntake.SERVO_DOWN);
        break;
      case OFF:
      default:
        servo.setVoltage(0);
        break;
    }
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void toggle(CoralIntakeStates state) {
    switch (state) {
      case REVERSE:
        toggle(Constants.CoralIntake.REVERSE_SPEED);
        break;
      case INTAKE:
        toggle(Constants.CoralIntake.INTAKE_SPEED);
        break;
      case OFF: // Off
      default: // should never happen
        toggle(0);
        break;
    }
  }

  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
  private double lastSpeed = 0;

  @Override
  public void periodic() {
    double pidOutput = pid.calculate(getWheelVelocity());
    double pidOutputServo = servoPID.calculate(getServoAngle());
    double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());
    double servoFeedforwardOutput =
        servoFeedForward.calculateWithVelocities(
            getServoAngle(), currentSetpoint.velocity, servoPID.getSetpoint().velocity);

    motor.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
    servo.setVoltage(pidOutputServo + servoFeedforwardOutput);

    lastSpeed = pid.getSetpoint();
    currentSetpoint = servoPID.getSetpoint();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutineWheel.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutineWheel.dynamic(direction);
  }

  @Override
  public void kill() {
    toggle(CoralIntakeStates.OFF);
    motor.setVoltage(0);
    servo.setVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() {
    motor.close();
    servo.close();
  }
}
