package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.subsystems.io.sim.ClimberSim;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends KillableSubsystem implements ShuffleboardPublisher, PoweredSubsystem {

  private final ClimberIO io;
  private final SysIdRoutine sysIdRoutine;
  private final MotionMagicVoltage armRequest;

  private ClimberState currentState = ClimberState.Park;

  public Climber(ClimberIO io) {
    this.io = io;

    var config = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = config.Slot0;
    slot0Configs.kS = Constants.Climber.kS;
    slot0Configs.kV = Constants.Climber.kV;
    slot0Configs.kA = Constants.Climber.kA;
    slot0Configs.kG = Constants.Climber.kG;
    slot0Configs.kP = Constants.Climber.kP;
    slot0Configs.kI = Constants.Climber.kI;
    slot0Configs.kD = Constants.Climber.kD;
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    config.Feedback.SensorToMechanismRatio = Constants.Climber.GEAR_RATIO;

    // set Motion Magic settings
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Climber.MAX_ARM_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Climber.MAX_ARM_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = 1600;

    io.applyTalonFXConfig(
        config
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Constants.Climber.SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimit(Constants.Climber.STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)));

    io.setPosition(Constants.Climber.START_ROTATIONS.in(Rotations));
    armRequest = new MotionMagicVoltage(Constants.Climber.START_ROTATIONS.in(Rotations));
    set(ClimberState.Park);

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2.0).per(Second),
                Volts.of(1.5),
                Seconds.of(1.3),
                (state -> Logger.recordOutput("Climber/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism((v) -> io.setVoltage(v.in(Volts)), null, this));

    SmartDashboard.putNumber("Climber", Constants.Climber.START_ROTATIONS.in(Rotations));
    SmartDashboard.putBoolean("Ratchet", false);
  }

  public enum ClimberState {
    Park,
    Extend,
    Climb
  }

  @Override
  public void periodic() {
    io.setRatchet(SmartDashboard.getBoolean("Ratchet", false) ? 1 : 0);

    // Update mechanism
    RobotContainer.model.climber.update(getRotations());
    RobotContainer.model.climber.updateSetpoint(getRotations()); // TODO add setpoint
  }

  public boolean atGoal() {
    if (currentState == ClimberState.Climb) {
      return getRotations() >= Constants.Climber.CLIMBED_ROTATIONS.in(Rotations);
    } else if (currentState == ClimberState.Extend) {
      return Math.abs(getRotations() - Constants.Climber.EXTENDED_ROTATIONS.in(Rotations)) < 0.01;
    } else {
      return Math.abs(getRotations() - Constants.Climber.PARK_ROTATIONS.in(Rotations)) < 0.01;
    }
  }

  public void set(ClimberState state) {
    currentState = state;
    switch (state) {
      case Park:
        io.setRatchet(0);
        // io.setMotionMagic(armRequest.withPosition(Constants.Climber.PARK_ROTATIONS.in(Rotations)));
        break;
      case Extend:
        io.setRatchet(0);
        io.setMotionMagic(
            armRequest.withPosition(Constants.Climber.EXTENDED_ROTATIONS.in(Rotations)));
        break;
      case Climb:
        io.setRatchet(1);
        break;
    }
  }

  @AutoLogOutput
  public ClimberState getCurrentState() {
    return currentState;
  }

  @AutoLogOutput
  public double getRotations() {
    return io.getPosition();
  }

  @AutoLogOutput
  public double getVelocity() {
    return io.getVelocity();
  }

  @AutoLogOutput
  public double getArmSetTo() {
    return io.getVoltage();
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public ClimberSim getSimIO() throws Exception {
    if (io instanceof ClimberSim) {
      return (ClimberSim) io;
    } else {
      throw new Exception("ClimberIO is not a simulation");
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void setupShuffleboard() {}

  @Override
  public void kill() {
    io.setVoltage(0);
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    io.close();
  }

  @Override
  public double getCurrentDrawAmps() {
    return io.getCurrentDrawAmps();
  }
}
