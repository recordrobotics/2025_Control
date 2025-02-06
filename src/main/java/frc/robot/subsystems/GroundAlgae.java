package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class GroundAlgae extends KillableSubsystem implements ShuffleboardPublisher {
  private SparkMax motor;
  private final SparkMax arm;

  private DigitalInput algaeDetector = new DigitalInput(RobotMap.GroundAlgae.LIMIT_SWITCH_ID);
  private static Boolean debounced_value = false;
  private Debouncer m_debouncer =
      new Debouncer(Constants.GroundAlgae.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

  private static final double defaultSpeed = Constants.GroundAlgae.DEFAULT_SPEED;

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
  private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();

  public GroundAlgae() {
    motor = new SparkMax(RobotMap.GroundAlgae.MOTOR_ID, MotorType.kBrushless);
    arm = new SparkMax(RobotMap.CoralIntake.ARM_ID, MotorType.kBrushless);
    toggle(GroundAlgaeStates.OFF);

    sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state -> Logger.recordOutput("GroundAlgae/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(motor::setVoltage, null, this));
  }

  private final SysIdRoutine sysIdRoutine;

  public enum GroundAlgaeStates {
    IN,
    OUT,
    OFF;
  }

  public enum GroundAlgaeArmStates {
    UP,
    DOWN,
    OFF;
  }

  public void toggle(GroundAlgaeStates state, double speed) {
    switch (state) {
      case IN: // take in note
        motor.set(speed);
        break;
      case OUT: // push out note
        motor.set(-speed);
        break;
      case OFF: // turn off or kill
      default: // should never happen
        motor.set(0);
        break;
    }
  }

  public void toggle(GroundAlgaeStates state) {
    toggle(state, defaultSpeed);
  }

  @AutoLogOutput
  public double getWheelPosition() {
    return motor.getEncoder().getPosition();
  }

  @AutoLogOutput
  public double getWheelVelocity() {
    return motor.getEncoder().getVelocity();
  }

  public double getArmAngle() {
    return arm.getEncoder().getPosition() * Math.PI * 2;
  }

  public void toggleArm(double pos) {
    armPID.setGoal(pos);
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

  public boolean armAtGoal() {
    return armPID.atGoal();
  }

  public boolean hasAlgae() {
    return debounced_value;
  }

  public void periodic() {
    debounced_value = !m_debouncer.calculate(algaeDetector.get());

    double pidOutputArm = armPID.calculate(getArmAngle());
    double armFeedforwardOutput =
        armFeedForward.calculateWithVelocities(
            getArmAngle(), currentSetpoint.velocity, armPID.getSetpoint().velocity);
    arm.setVoltage(pidOutputArm + armFeedforwardOutput);
    currentSetpoint = armPID.getSetpoint();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void kill() {
    toggle(GroundAlgaeStates.OFF);
    motor.setVoltage(0);
    arm.setVoltage(0);
  }

  /** frees up all hardware allocations */
  @Override
  public void close() {
    motor.close();
    arm.close();
    algaeDetector.close();
  }

  @Override
  public void setupShuffleboard() {
    DashboardUI.Test.addSlider("Ground Algae Arm Pos", arm.getEncoder().getPosition(), -1, 1)
        .subscribe(this::toggleArm);
    // TODO more shuffleboard stuff
    DashboardUI.Test.addSlider("Ground Algae", motor.get(), -1, 1).subscribe(motor::set);
  }
}
