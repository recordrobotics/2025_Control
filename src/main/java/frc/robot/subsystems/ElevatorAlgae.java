// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.ShuffleboardPublisher;

public class ElevatorAlgae extends KillableSubsystem implements ShuffleboardPublisher {
  private final DigitalInput algaeDetector =
      new DigitalInput(RobotMap.ElevatorAlgae.LIMIT_SWITCH_ID);
  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.CoralIntake.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private final SparkMax motor;
  private final PIDController pid =
      new PIDController(
          Constants.ElevatorAlgae.kP, Constants.ElevatorAlgae.kI, Constants.ElevatorAlgae.kD);
  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(Constants.ElevatorAlgae.kS, Constants.ElevatorAlgae.kV);

  public ElevatorAlgae() {
    motor = new SparkMax(RobotMap.ElevatorAlgae.MOTOR_ID, MotorType.kBrushless);
    toggle(ElevatorAlgaeStates.OFF); // initialize as off
  }

  public enum ElevatorAlgaeStates {
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
  public void toggle(ElevatorAlgaeStates state) {
    switch (state) {
      case OUT:
        toggle(Constants.ElevatorAlgae.OUT_SPEED);
        break;
      case INTAKE:
        toggle(Constants.ElevatorAlgae.INTAKE_SPEED);
        break;
      case OFF: // Off
      default: // should never happen
        toggle(0);
        break;
    }
  }

  public boolean hasAlgae() {
    return debounced_value;
  }

  @Override
  public void periodic() {
    double pidOutput = pid.calculate(getWheelVelocity());
    double feedforwardOutput = feedForward.calculate(pid.getSetpoint());
    motor.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control

    debounced_value = !m_debouncer.calculate(algaeDetector.get());
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Algae on the Elevator", motor.get(), -1, 1).subscribe(motor::set);
  }

  @Override
  public void kill() {
    toggle(ElevatorAlgaeStates.OFF);
    motor.setVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() {
    motor.close();
    algaeDetector.close();
  }
}
