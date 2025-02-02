// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends KillableSubsystem implements ShuffleboardPublisher {
  private final SparkMax motor;
  private final TalonFX arm;

  private final DigitalInput coralDetector = new DigitalInput(RobotMap.CoralIntake.LIMIT_SWITCH_ID);
  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.CoralIntake.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private final ProfiledPIDController armPID =
      new ProfiledPIDController(
          Constants.CoralIntake.sP,
          Constants.CoralIntake.sI,
          Constants.CoralIntake.sD,
          new Constraints(
              Constants.CoralIntake.MAX_ARM_VELOCITY,
              Constants.CoralIntake.MAX_ARM_ACCELERATION));
  private final ArmFeedforward armFeedForward =
      new ArmFeedforward(
          Constants.CoralIntake.sS,
          Constants.CoralIntake.sG,
          Constants.CoralIntake.sV,
          Constants.CoralIntake.sA);

  private final PIDController pid =
      new PIDController(
          Constants.CoralIntake.kP, Constants.CoralIntake.kI, Constants.CoralIntake.kD);

  private final SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(Constants.CoralIntake.kS, Constants.CoralIntake.kV);

  public CoralIntake() {
    motor = new SparkMax(RobotMap.CoralIntake.MOTOR_ID, MotorType.kBrushless);
    arm = new TalonFX(RobotMap.CoralIntake.ARM_ID);
    toggle(CoralIntakeStates.OFF);

    sysIdRoutineWheel =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // default 1 volt/second ramp rate
                null, // default 7 volt step voltage
                null,
                (state ->
                    Logger.recordOutput("CoralIntake/Wheel/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(motor::setVoltage, null, this));

    sysIdRoutineArm =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2).per(Second),
                Volts.of(0.6),
                Seconds.of(3),
                (state ->
                    Logger.recordOutput("CoralIntake/Arm/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism((v) -> arm.setVoltage(v.magnitude()), null, this));
  }

  private final SysIdRoutine sysIdRoutineWheel;
  private final SysIdRoutine sysIdRoutineArm;

  public enum CoralIntakeStates {
    REVERSE,
    INTAKE,
    OFF;
  }

  public boolean hasCoral() {
    return debounced_value;
  }

  public enum IntakeArmStates {
    UP,
    DOWN,
    OFF;
  }

  @AutoLogOutput
  public double getWheelVelocity() {
    return motor.getEncoder().getVelocity() / 60.0; /* RPM -> RPS */
  }

  @AutoLogOutput
  public double getWheelPosition() {
    return motor.getEncoder().getPosition() / 60.0; /* RPM -> RPS */
  }

  @AutoLogOutput
  public double getArmAngle() {
    return arm.getPosition().getValueAsDouble() / Constants.CoralIntake.ARM_GEAR_RATIO;
  }

  @AutoLogOutput
  public double getArmVelocity() {
    return arm.getVelocity().getValueAsDouble() / Constants.CoralIntake.ARM_GEAR_RATIO;
  }

  /** Set the current shooter speed on both wheels to speed */
  public void toggle(double speed) {
    pid.setSetpoint(speed);
  }

  public void toggleArm(double pos) {
    armPID.setGoal(pos);
  }

  public boolean atGoal() {
    return armPID.atGoal();
  }

  public void toggleArm(IntakeArmStates state) {
    switch (state) {
      case UP:
        toggleArm(Constants.CoralIntake.ARM_UP);
        break;
      case DOWN:
        toggleArm(Constants.CoralIntake.ARM_DOWN);
        break;
      case OFF:
      default:
        arm.setVoltage(0);
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
  private double lastSpeed = 0; // TODO why is this here?

  @Override
  public void periodic() {
    double pidOutput = pid.calculate(getWheelVelocity());
    double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());
    motor.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
    lastSpeed = pid.getSetpoint();

    double pidOutputArm = armPID.calculate(getArmAngle());
    double armFeedforwardOutput =
        armFeedForward.calculateWithVelocities(
            getArmAngle(), currentSetpoint.velocity, armPID.getSetpoint().velocity);
    arm.setVoltage(pidOutputArm + armFeedforwardOutput);
    currentSetpoint = armPID.getSetpoint();

    // Update mechanism
    RobotContainer.model.coralIntake.update(getArmAngle());
    RobotContainer.model.coralIntake.updateSetpoint(currentSetpoint.position);

    debounced_value = !m_debouncer.calculate(coralDetector.get());
  }

  public Command sysIdQuasistaticWheel(SysIdRoutine.Direction direction) {
    return sysIdRoutineWheel.quasistatic(direction);
  }

  public Command sysIdDynamicWheel(SysIdRoutine.Direction direction) {
    return sysIdRoutineWheel.dynamic(direction);
  }

  public Command sysIdQuasistaticArm(SysIdRoutine.Direction direction) {
    return sysIdRoutineArm.quasistatic(direction);
  }

  public Command sysIdDynamicArm(SysIdRoutine.Direction direction) {
    return sysIdRoutineArm.dynamic(direction);
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Coral Intake Motor", motor.get(), -1, 1).subscribe(motor::set);
    DashboardUI.Test.addSlider(
            "Coral Intake Arm Pos", arm.getPosition().getValueAsDouble(), -1, 1)
        .subscribe(this::toggleArm);
  }

  @Override
  public void kill() {
    toggle(CoralIntakeStates.OFF);
    motor.setVoltage(0);
    arm.setVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() {
    motor.close();
    arm.close();
    coralDetector.close();
  }
}
