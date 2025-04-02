package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.ElevatorArmIO;
import frc.robot.subsystems.io.sim.ElevatorArmSim;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorArm extends KillableSubsystem
    implements ShuffleboardPublisher, PoweredSubsystem {

  private final ElevatorArmIO io;
  private final SysIdRoutine sysIdRoutine;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          Constants.ElevatorArm.kP,
          Constants.ElevatorArm.kI,
          Constants.ElevatorArm.kD,
          new TrapezoidProfile.Constraints(
              Constants.ElevatorArm.MAX_ARM_VELOCITY, Constants.ElevatorArm.MAX_ARM_ACCELERATION));
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          Constants.ElevatorArm.kS,
          Constants.ElevatorArm.kG,
          Constants.ElevatorArm.kV,
          Constants.ElevatorArm.kA);

  public ElevatorArm(ElevatorArmIO io) {
    this.io = io;

    io.applyArmTalonFXConfig(
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Constants.ElevatorArm.ARM_SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimit(Constants.ElevatorArm.ARM_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)));

    io.setArmPosition(
        Constants.ElevatorArm.ARM_GEAR_RATIO
            * Units.radiansToRotations(Constants.ElevatorArm.START_POS));
    toggle(ElevatorHeight.BOTTOM.getArmAngle());

    pid.setTolerance(0.15, 1.05);

    pid.reset(getArmAngle());

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2.0).per(Second),
                Volts.of(1.5),
                Seconds.of(1.3),
                (state -> Logger.recordOutput("ElevatorArm/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism((v) -> io.setArmVoltage(v.in(Volts)), null, this));

    SmartDashboard.putNumber("ElevatorArm", Constants.ElevatorArm.START_POS);
  }

  public ElevatorArmSim getSimIO() throws Exception {
    if (io instanceof ElevatorArmSim) {
      return (ElevatorArmSim) io;
    } else {
      throw new Exception("ElevatorArmIO is not a simulation");
    }
  }

  @AutoLogOutput
  public double getArmAngle() {
    return io.getArmPosition() / Constants.ElevatorArm.ARM_GEAR_RATIO * 2 * Math.PI;
  }

  @AutoLogOutput
  public double getArmVelocity() {
    return io.getArmVelocity() / Constants.ElevatorArm.ARM_GEAR_RATIO * 2 * Math.PI;
  }

  @AutoLogOutput
  public double getArmSetTo() {
    return io.getArmVoltage();
  }

  public void toggle(double angleRadians) {
    pid.setGoal(angleRadians);
  }

  public boolean atGoal() {
    return pid.atGoal();
  }

  public void setGoal(TrapezoidProfile.State goal) {
    pid.setGoal(goal);
  }

  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();

  @Override
  public void periodic() {
    // toggle(SmartDashboard.getNumber("ElevatorArm", Constants.ElevatorArm.START_POS));

    double pidOutputArm = pid.calculate(getArmAngle());

    double feedforwardOutput =
        feedforward.calculateWithVelocities(
            getArmAngle(), currentSetpoint.velocity, pid.getSetpoint().velocity);

    // Logger.recordOutput("ElevatorArmTargetPosition", pid.getSetpoint().position);
    // Logger.recordOutput("ElevatorArmTargetVelocity", pid.getSetpoint().velocity);
    // Logger.recordOutput("ElevatorArmSetVoltage", pidOutputArm);
    // Logger.recordOutput("ElevatorArmSetVoltageFF", feedforwardOutput);

    io.setArmVoltage(pidOutputArm + feedforwardOutput);
    currentSetpoint = pid.getSetpoint();

    // Update mechanism
    RobotContainer.model.elevatorArm.update(getArmAngle());
    RobotContainer.model.elevatorArm.updateSetpoint(currentSetpoint.position);
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Elevator Arm Pos", io.getArmPosition(), -1, 1)
        .subscribe(this::toggle);
  }

  @Override
  public void kill() {
    io.setArmVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    io.close();
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getArmCurrentDrawAmps();
  }
}
