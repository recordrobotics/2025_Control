// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.CoralIntakeIO;
import frc.robot.subsystems.io.sim.CoralIntakeSim;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends KillableSubsystem
    implements ShuffleboardPublisher, PoweredSubsystem {

  private final CoralIntakeIO io;

  private final ProfiledPIDController armPID =
      new ProfiledPIDController(
          Constants.CoralIntake.sP,
          Constants.CoralIntake.sI,
          Constants.CoralIntake.sD,
          new Constraints(
              Constants.CoralIntake.MAX_ARM_VELOCITY, Constants.CoralIntake.MAX_ARM_ACCELERATION));
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
      new SimpleMotorFeedforward(
          Constants.CoralIntake.kS, Constants.CoralIntake.kV, Constants.CoralIntake.kA);

  private CoralIntakeState currentIntakeState = CoralIntakeState.UP;

  private double intakePushAndPullRampStart = 0;

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;

    io.applyArmTalonFXConfig(
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withReverseSoftLimitThreshold(
                        Constants.CoralIntake.ARM_GEAR_RATIO
                            * Units.radiansToRotations(Constants.CoralIntake.ARM_DOWN))
                    .withReverseSoftLimitEnable(true))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Constants.CoralIntake.ARM_SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimit(Constants.CoralIntake.ARM_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)));

    io.setArmPosition(
        Constants.CoralIntake.ARM_GEAR_RATIO
            * Units.radiansToRotations(Constants.CoralIntake.ARM_START_POS));
    set(CoralIntakeState.UP);

    armPID.setTolerance(0.15, 1.05);

    armPID.reset(getArmAngle());

    sysIdRoutineWheel =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // default 1 volt/second ramp rate
                null, // default 7 volt step voltage
                null,
                (state ->
                    Logger.recordOutput("CoralIntake/Wheel/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism((v) -> io.setWheelVoltage(v.in(Volts)), null, this));

    sysIdRoutineArm =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(4.0).per(Second),
                Volts.of(2.3),
                Seconds.of(1.0),
                (state -> Logger.recordOutput("CoralIntake/Arm/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism((v) -> io.setArmVoltage(v.in(Volts)), null, this));

    SmartDashboard.putNumber("CoralIntakeArm", Constants.CoralIntake.ARM_START_POS);
  }

  public CoralIntakeSim getSimIO() throws Exception {
    if (io instanceof CoralIntakeSim) {
      return (CoralIntakeSim) io;
    } else {
      throw new Exception("CoralIntakeIO is not a simulation");
    }
  }

  private final SysIdRoutine sysIdRoutineWheel;
  private final SysIdRoutine sysIdRoutineArm;

  public enum CoralIntakeState {
    UP,
    SOURCE,
    PUSH_READY,
    PUSH_OUT,
    GROUND,
    L1_SCORE,
    L1_DOWN;
  }

  @AutoLogOutput
  public double getWheelVelocity() {
    return io.getWheelVelocity() / 60.0 / Constants.CoralIntake.WHEEL_GEAR_RATIO; /* RPM -> RPS */
  }

  @AutoLogOutput
  public double getWheelPosition() {
    return io.getWheelPosition() / Constants.CoralIntake.WHEEL_GEAR_RATIO;
  }

  @AutoLogOutput
  public double getWheelSetTo() {
    return io.getWheelVoltage();
  }

  @AutoLogOutput
  public double getArmAngle() {
    return io.getArmPosition() / Constants.CoralIntake.ARM_GEAR_RATIO * 2 * Math.PI;
  }

  @AutoLogOutput
  public double getArmVelocity() {
    return io.getArmVelocity() / Constants.CoralIntake.ARM_GEAR_RATIO * 2 * Math.PI;
  }

  @AutoLogOutput
  public double getArmSetTo() {
    return io.getArmVoltage();
  }

  public CoralIntakeState getState() {
    return currentIntakeState;
  }

  /** Set the current shooter speed on both wheels to speed */
  public void setWheel(double speed) {
    pid.setSetpoint(speed);
  }

  public void setArm(double angleRadians) {
    armPID.setGoal(angleRadians);
  }

  public boolean armAtGoal() {
    return armPID.atGoal();
  }

  public void set(CoralIntakeState state) {
    currentIntakeState = state;
    switch (state) {
      case SOURCE:
        setWheel(Constants.CoralIntake.SOURCE_SPEED);
        setArm(Constants.CoralIntake.ARM_INTAKE);
        break;
      case PUSH_READY:
        setWheel(0);
        setArm(Constants.CoralIntake.ARM_PUSH);
      case PUSH_OUT:
        intakePushAndPullRampStart = Timer.getTimestamp();
        setWheel(Constants.CoralIntake.PUSH_OUT_SPEED);
        setArm(Constants.CoralIntake.ARM_PUSH);
        break;
      case GROUND:
        setWheel(Constants.CoralIntake.INTAKE_SPEED);
        setArm(Constants.CoralIntake.ARM_DOWN);
        break;
      case L1_SCORE:
        setWheel(Constants.CoralIntake.L1_SCORE_SPEED);
        setArm(Constants.CoralIntake.ARM_SCORE_L1);
        break;
      case L1_DOWN:
        setWheel(0);
        setArm(Constants.CoralIntake.ARM_SCORE_L1);
        break;
      case UP:
        setWheel(0);
        setArm(Constants.CoralIntake.ARM_UP);
        break;
      default: // should never happen
        io.setArmVoltage(0);
        setWheel(0);
        break;
    }
  }

  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
  private double lastSpeed = 0;

  @Override
  public void periodic() {
    // toggleArm(SmartDashboard.getNumber("CoralIntakeArm", Constants.CoralIntake.ARM_START_POS));

    if (currentIntakeState == CoralIntakeState.PUSH_OUT) {
      // push and pull ramp
      double rampT =
          MathUtil.clamp(
              (Timer.getTimestamp() - intakePushAndPullRampStart)
                  / Constants.CoralIntake.PUSH_OUT_RAMP_TIME,
              0,
              1);
      double rampedSpeed =
          MathUtil.interpolate(
              Constants.CoralIntake.PUSH_OUT_SPEED,
              Constants.CoralIntake.PULL_THROUGH_SPEED,
              rampT);
      setWheel(rampedSpeed);
    }

    double pidOutput = pid.calculate(getWheelVelocity());
    double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());
    io.setWheelVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
    lastSpeed = pid.getSetpoint();

    double pidOutputArm = armPID.calculate(getArmAngle());

    double armFeedforwardOutput =
        armFeedForward.calculateWithVelocities(
            getArmAngle(), currentSetpoint.velocity, armPID.getSetpoint().velocity);

    // Logger.recordOutput("CoralArmTargetPosition", armPID.getSetpoint().position);
    // Logger.recordOutput("CoralArmTargetVelocity", armPID.getSetpoint().velocity);
    // Logger.recordOutput("CoralIntakeSetVoltage", pidOutputArm);
    // Logger.recordOutput("CoralIntakeSetVoltageFF", armFeedforwardOutput);

    io.setArmVoltage(pidOutputArm + armFeedforwardOutput);
    currentSetpoint = armPID.getSetpoint();

    // Update mechanism
    RobotContainer.model.coralIntake.update(getArmAngle());
    RobotContainer.model.coralIntake.updateSetpoint(currentSetpoint.position);
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
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
    DashboardUI.Test.addSlider("Coral Intake Motor", io.getWheelPercent(), -1, 1)
        .subscribe(io::setWheelPercent);
    DashboardUI.Test.addSlider(
            "Coral Intake Arm Pos",
            io.getArmPosition(),
            Constants.CoralIntake.ARM_DOWN,
            Constants.CoralIntake.ARM_UP)
        .subscribe(this::setArm);
  }

  @Override
  public void kill() {
    setWheel(0);
    // io.setWheelVoltage(0);
    // io.setArmVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    io.close();
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getWheelCurrentDrawAmps() + io.getArmCurrentDrawAmps();
  }
}
