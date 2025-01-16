// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.KillableSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.ShuffleboardPublisher;

public class CoralShooter extends KillableSubsystem implements ShuffleboardPublisher {
  private final DigitalInput coralDetector =
      new DigitalInput(RobotMap.CoralShooter.LIMIT_SWITCH_ID);
  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.CoralIntake.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private SparkMax motor;
  private RelativeEncoder encoder;
  private final PIDController pid =
      new PIDController(
          Constants.CoralShooter.kP, Constants.CoralShooter.kI, Constants.CoralShooter.kD);
  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(Constants.CoralShooter.kS, Constants.CoralShooter.kV);

  public CoralShooter() {
    motor = new SparkMax(RobotMap.CoralShooter.MOTOR_ID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    toggle(CoralShooterStates.OFF); // initialize as off
  }
  
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              motor::setVoltage,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("coral-shooter-wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            motor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(encoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(encoder.getVelocity()/60, RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  public enum CoralShooterStates {
    OUT,
    INTAKE,
    OFF;
  }
  
  public double getWheelVelocity() {
    return motor.getEncoder().getVelocity() / 60.0; /* RPM -> RPS */
  }
  
  /** Set the current shooter speed on both wheels to speed */
  public void toggle(double speed) {
    pid.setSetpoint(speed);
  }

  /** Set the shooter speed to the preset ShooterStates state */
  public void toggle(CoralShooterStates state) {
    switch (state) {
      case OUT:
      toggle(Constants.CoralShooter.OUT_SPEED);
      break;
      case INTAKE:
      toggle(Constants.CoralShooter.INTAKE_SPEED);
      break;
      case OFF: // Off
      default: // should never happen
      toggle(0);
      break;
    }
  }
  
  public boolean hasCoral() {
    return debounced_value;
  }
  
  @Override
  public void periodic() {
    double pidOutput = pid.calculate(getWheelVelocity());
    double feedforwardOutput = feedForward.calculate(pid.getSetpoint());
    motor.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control

    debounced_value = !m_debouncer.calculate(coralDetector.get());
  }
  
  @Override
  public void setupShuffleboard() {
    ShuffleboardUI.Test.addSlider("Coral Shooter", motor.get(), -1, 1).subscribe(motor::set);
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Coral Shooter", motor.get(), -1, 1).subscribe(motor::set);
  }

  @Override
  public void kill() {
    toggle(CoralShooterStates.OFF);
    motor.setVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() {
    motor.close();
    coralDetector.close();
  }
}
