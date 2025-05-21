package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import frc.robot.utils.SimpleMath;
import org.littletonrobotics.junction.Logger;

public class ElevatorArm extends KillableSubsystem
    implements ShuffleboardPublisher, PoweredSubsystem {

  private final ElevatorArmIO io;
  private final SysIdRoutine sysIdRoutine;

  private final MotionMagicExpoVoltage armRequest;

  public ElevatorArm(ElevatorArmIO io) {
    this.io = io;

    var armConfig = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs_arm = armConfig.Slot0;
    slot0Configs_arm.kS = Constants.ElevatorArm.kS;
    slot0Configs_arm.kV = Constants.ElevatorArm.kV;
    slot0Configs_arm.kA = Constants.ElevatorArm.kA;
    slot0Configs_arm.kG = Constants.ElevatorArm.kG;
    slot0Configs_arm.kP = Constants.ElevatorArm.kP;
    slot0Configs_arm.kI = Constants.ElevatorArm.kI;
    slot0Configs_arm.kD = Constants.ElevatorArm.kD;
    slot0Configs_arm.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Feedback.SensorToMechanismRatio = Constants.ElevatorArm.ARM_GEAR_RATIO;

    // set Motion Magic settings
    var motionMagicConfigs_arm = armConfig.MotionMagic;
    motionMagicConfigs_arm.MotionMagicCruiseVelocity = Constants.ElevatorArm.MAX_ARM_VELOCITY;
    motionMagicConfigs_arm.MotionMagicAcceleration = Constants.ElevatorArm.MAX_ARM_ACCELERATION;
    motionMagicConfigs_arm.MotionMagicJerk = 1600;
    motionMagicConfigs_arm.MotionMagicExpo_kV = 1.931;
    motionMagicConfigs_arm.MotionMagicExpo_kA = 1.1;

    io.applyArmTalonFXConfig(
        armConfig
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

    io.setArmPosition(Units.radiansToRotations(Constants.ElevatorArm.START_POS));
    armRequest =
        new MotionMagicExpoVoltage(Units.radiansToRotations(Constants.ElevatorArm.START_POS));
    set(ElevatorHeight.BOTTOM.getArmAngle());

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

  @AutoLogLevel(level = Level.Sysid)
  public double getArmAngle() {
    return io.getArmPosition() * 2 * Math.PI;
  }

  @AutoLogLevel(level = Level.Sysid)
  public double getArmVelocity() {
    return io.getArmVelocity() * 2 * Math.PI;
  }

  /** Used for sysid as units have to be in rotations in the logs */
  @AutoLogLevel(level = Level.Sysid)
  public double getArmAngleRotations() {
    return io.getArmPosition();
  }

  @AutoLogLevel(level = Level.Sysid)
  public double getArmVelocityRotations() {
    return io.getArmVelocity();
  }

  @AutoLogLevel(level = Level.Sysid)
  public double getArmSetTo() {
    return io.getArmVoltage();
  }

  public void set(double angleRadians) {
    currentSetpoint.position = angleRadians;
    io.setArmMotionMagic(armRequest.withPosition(Units.radiansToRotations(angleRadians)));
  }

  public boolean atGoal() {
    return SimpleMath.isWithinTolerance(getArmAngle(), currentSetpoint.position, 0.15)
        && SimpleMath.isWithinTolerance(getArmVelocity(), 0, 1.05);
  }

  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();

  @Override
  public void periodic() {
    set(SmartDashboard.getNumber("ElevatorArm", Constants.ElevatorArm.START_POS));

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
    DashboardUI.Test.addSlider("Elevator Arm Pos", io.getArmPosition(), -1, 1).subscribe(this::set);
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
