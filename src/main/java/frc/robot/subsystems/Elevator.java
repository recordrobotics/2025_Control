package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.ElevatorIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.EncoderResettableSubsystem;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import frc.robot.utils.SysIdManager;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class Elevator extends KillableSubsystem
        implements ShuffleboardPublisher, PoweredSubsystem, EncoderResettableSubsystem {
    private final ElevatorIO io;

    private final MotionMagicExpoVoltage elevatorRequest;
    private final Follower elevatorFollower;

    private double leadPositionCached = 0;
    private double leadVelocityCached = 0;
    private double leadVoltageCached = 0;

    public Elevator(ElevatorIO io) {
        this.io = io;

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0Configs_elevator = elevatorConfig.Slot0;
        slot0Configs_elevator.kS = Constants.Elevator.kS;
        slot0Configs_elevator.kV = Constants.Elevator.kV;
        slot0Configs_elevator.kA = Constants.Elevator.kA;
        slot0Configs_elevator.kG = Constants.Elevator.kG;
        slot0Configs_elevator.kP = Constants.Elevator.kP;
        slot0Configs_elevator.kI = Constants.Elevator.kI;
        slot0Configs_elevator.kD = Constants.Elevator.kD;
        slot0Configs_elevator.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Feedback.SensorToMechanismRatio = 1.0 / Constants.Elevator.METERS_PER_ROTATION;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigs_elevator = elevatorConfig.MotionMagic;
        motionMagicConfigs_elevator.MotionMagicCruiseVelocity = Constants.Elevator.kMaxVelocity;
        motionMagicConfigs_elevator.MotionMagicAcceleration = Constants.Elevator.kMaxAcceleration;
        motionMagicConfigs_elevator.MotionMagicJerk = 1600;
        motionMagicConfigs_elevator.MotionMagicExpo_kV = 5.8;
        motionMagicConfigs_elevator.MotionMagicExpo_kA = 1.5;

        io.applyTalonFXConfig(elevatorConfig
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.Elevator.SUPPLY_CURRENT_LIMIT)
                        .withSupplyCurrentLowerLimit(Constants.Elevator.SUPPLY_CURRENT_LOWER_LIMIT)
                        .withSupplyCurrentLowerTime(1)
                        .withStatorCurrentLimit(Constants.Elevator.STATOR_CURRENT_LIMIT)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true)));

        leadPositionCached = io.getLeadMotorPosition();

        elevatorRequest = new MotionMagicExpoVoltage(Constants.Elevator.STARTING_HEIGHT);
        elevatorFollower = io.createFollower();

        io.setFollowerMotionMagic(elevatorFollower);

        set(Constants.Elevator.STARTING_HEIGHT);

        sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        Volts.of(4.5).per(Second),
                        Volts.of(3.0),
                        Seconds.of(1.2),
                        (state -> Logger.recordOutput("Elevator/SysIdTestState", state.toString()))),
                new SysIdRoutine.Mechanism(v -> io.setLeadMotorVoltage(v.in(Volts)), null, this));

        SmartDashboard.putNumber("Elevator", Constants.Elevator.STARTING_HEIGHT);
    }

    private final SysIdRoutine sysIdRoutine;
    private double m_setpoint;

    /** Height of the elevator in meters */
    @AutoLogLevel(level = Level.Sysid)
    public double getCurrentHeight() {
        return leadPositionCached;
    }

    @AutoLogLevel(level = Level.Sysid)
    public double getCurrentVelocity() {
        return leadVelocityCached;
    }

    @AutoLogLevel(level = Level.Sysid)
    public double getCurrentVoltage() {
        return leadVoltageCached;
    }

    @AutoLogLevel(level = Level.DebugReal)
    private boolean getBottomEndStopPressed() {
        return io.getBottomEndStop();
    }

    @AutoLogLevel(level = Level.DebugReal)
    private boolean getTopEndStopPressed() {
        return io.getTopEndStop();
    }

    @Override
    public void periodicManaged() {

        leadPositionCached = io.getLeadMotorPosition();
        leadVelocityCached = io.getLeadMotorVelocity();
        if (Level.Sysid.isAtLeast(Constants.RobotState.AUTO_LOG_LEVEL)) {
            leadVoltageCached = io.getLeadMotorVoltage();
        }

        // set(SmartDashboard.getNumber("Elevator", Constants.Elevator.STARTING_HEIGHT));

        // Update mechanism
        RobotContainer.model.elevator.update(getCurrentHeight());
        RobotContainer.model.elevator.updateSetpoint(m_setpoint);
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    public void set(double heightMeters) {
        m_setpoint = heightMeters;

        if (SysIdManager.getSysIdRoutine() != SysIdManager.SysIdRoutine.Elevator) {
            io.setLeadMotionMagic(elevatorRequest.withPosition(heightMeters));
        }
    }

    public void moveTo(ElevatorHeight height) {
        set(height.getHeight());
    }

    @AutoLogLevel(level = Level.Real)
    public ElevatorHeight getNearestHeight() {
        double currentHeight = getCurrentHeight();
        double currentArmAngle = RobotContainer.elevatorArm.getArmAngle();

        ElevatorHeight[] heights = ElevatorHeight.values();
        Arrays.sort(
                heights,
                (a, b) -> Double.compare(
                        a.getDifference(currentHeight, currentArmAngle),
                        b.getDifference(currentHeight, currentArmAngle)));
        return heights[0];
    }

    public boolean atGoal() {
        return Math.abs(m_setpoint - getCurrentHeight()) < Constants.Elevator.AT_GOAL_POSITION_TOLERANCE
                && Math.abs(getCurrentVelocity()) < Constants.Elevator.AT_GOAL_VELOCITY_TOLERANCE;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void kill() {}

    @Override
    public void close() throws Exception {
        io.close();
    }

    @Override
    public void setupShuffleboard() {
        DashboardUI.Test.addSlider("Elevator Target", m_setpoint, 0, ElevatorHeight.L4.getHeight())
                .subscribe(this::set);
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getLeadMotorCurrentDraw() + io.getFollowerMotorCurrentDraw();
    }

    @Override
    public void resetEncoders() {
        io.setLeadMotorPosition(Constants.Elevator.STARTING_HEIGHT);
        io.setFollowerMotorPosition(Constants.Elevator.STARTING_HEIGHT);

        leadPositionCached = Constants.Elevator.STARTING_HEIGHT;
    }
}
