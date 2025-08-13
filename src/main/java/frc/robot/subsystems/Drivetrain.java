package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.io.real.SwerveModuleReal;
import frc.robot.subsystems.io.sim.SwerveModuleSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.DCMotors;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.modifiers.DrivetrainControl;
import frc.robot.utils.modifiers.IDrivetrainControlModifier;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends KillableSubsystem implements ShuffleboardPublisher, PoweredSubsystem {
    // Creates swerve module objects
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    private final SysIdRoutine sysIdRoutineDriveMotorsSpin;
    private final SysIdRoutine sysIdRoutineDriveMotorsForward;
    private final SysIdRoutine sysIdRoutineTurnMotors;

    // Creates swerve kinematics
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            Constants.Swerve.frontLeftConstants.wheelLocation,
            Constants.Swerve.frontRightConstants.wheelLocation,
            Constants.Swerve.backLeftConstants.wheelLocation,
            Constants.Swerve.backRightConstants.wheelLocation);

    // Create and configure a drivetrain simulation configuration
    private final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
            // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(() -> new GyroSimulation(0.5, 0.05)) // navX-Micro
            // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                    DCMotors.getKrakenX44(1), // Steer motor is a Kraken X44
                    Constants.Swerve.KRAKEN_DRIVE_GEAR_RATIO, // Drive motor gear ratio.
                    Constants.Swerve.KRAKEN_TURN_GEAR_RATIO, // Steer motor gear ratio.
                    Volts.of(0.1),
                    Volts.of(0.2),
                    Inches.of(2),
                    KilogramSquareMeters.of(0.03),
                    COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof // Use the COF for Neoprene Tread
                    ))
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(
                    Meters.of(Constants.Frame.ROBOT_WHEEL_DISTANCE_LENGTH),
                    Meters.of(Constants.Frame.ROBOT_WHEEL_DISTANCE_WIDTH))
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(
                    Meters.of(Constants.Frame.FRAME_WITH_BUMPER_WIDTH),
                    Meters.of(Constants.Frame.FRAME_WITH_BUMPER_WIDTH))
            .withRobotMass(Kilograms.of(Constants.Frame.ROBOT_MASS));

    private final SwerveDriveSimulation swerveDriveSimulation;

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    private int lastModifiersAppliedCount = 0;
    private SwerveModuleState[] lastModuleSetpoints = new SwerveModuleState[0];

    public final List<IDrivetrainControlModifier> modifiers = new ArrayList<>();

    public SwerveModule getFrontLeftModule() {
        return m_frontLeft;
    }

    public SwerveModule getFrontRightModule() {
        return m_frontRight;
    }

    public SwerveModule getBackLeftModule() {
        return m_backLeft;
    }

    public SwerveModule getBackRightModule() {
        return m_backRight;
    }

    public SwerveDriveSimulation getSwerveDriveSimulation() {
        return swerveDriveSimulation;
    }

    public Drivetrain() {
        if (Constants.RobotState.getMode() == Mode.REAL) {
            swerveDriveSimulation = null;
            m_frontLeft = new SwerveModule(
                    Constants.Swerve.frontLeftConstants,
                    new SwerveModuleReal(Constants.Swerve.kDt, Constants.Swerve.frontLeftConstants));
            m_frontRight = new SwerveModule(
                    Constants.Swerve.frontRightConstants,
                    new SwerveModuleReal(Constants.Swerve.kDt, Constants.Swerve.frontRightConstants));
            m_backLeft = new SwerveModule(
                    Constants.Swerve.backLeftConstants,
                    new SwerveModuleReal(Constants.Swerve.kDt, Constants.Swerve.backLeftConstants));
            m_backRight = new SwerveModule(
                    Constants.Swerve.backRightConstants,
                    new SwerveModuleReal(Constants.Swerve.kDt, Constants.Swerve.backRightConstants));
        } else {
            /* Create a swerve drive simulation */
            swerveDriveSimulation = new SwerveDriveSimulation(
                    // Specify Configuration
                    driveTrainSimulationConfig,
                    // Specify starting pose
                    DashboardUI.Autonomous.getStartingLocation().getPose());

            // Register the drivetrain simulation to the default simulation world
            SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

            m_frontLeft = new SwerveModule(
                    Constants.Swerve.frontLeftConstants,
                    new SwerveModuleSim(swerveDriveSimulation.getModules()[0], Constants.Swerve.frontLeftConstants));
            m_frontRight = new SwerveModule(
                    Constants.Swerve.frontRightConstants,
                    new SwerveModuleSim(swerveDriveSimulation.getModules()[1], Constants.Swerve.frontRightConstants));
            m_backLeft = new SwerveModule(
                    Constants.Swerve.backLeftConstants,
                    new SwerveModuleSim(swerveDriveSimulation.getModules()[2], Constants.Swerve.backLeftConstants));
            m_backRight = new SwerveModule(
                    Constants.Swerve.backRightConstants,
                    new SwerveModuleSim(swerveDriveSimulation.getModules()[3], Constants.Swerve.backRightConstants));
        }

        setpointGenerator = new SwerveSetpointGenerator(
                Constants.Swerve.PPDefaultConfig,
                Units.rotationsToRadians(
                        Constants.Swerve.TurnMaxAngularVelocity) // The max rotation velocity of a swerve module in
                // radians per second
                );

        // Initialize the previous setpoint to the robot's current speeds & module states
        ChassisSpeeds currentSpeeds = getChassisSpeeds();
        SwerveModuleState[] currentStates = getModuleStates();
        previousSetpoint = new SwerveSetpoint(
                currentSpeeds, currentStates, DriveFeedforwards.zeros(Constants.Swerve.PPDefaultConfig.numModules));

        sysIdRoutineDriveMotorsSpin = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        Volts.of(3.0).per(Second),
                        Volts.of(3.0),
                        Seconds.of(1.5),
                        (state -> Logger.recordOutput("Drivetrain/Drive/SysIdTestState", state.toString()))),
                new SysIdRoutine.Mechanism(this::SysIdOnlyDriveMotorsSpin, null, this));

        sysIdRoutineDriveMotorsForward = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        Volts.of(3.0).per(Second),
                        Volts.of(3.0),
                        Seconds.of(1.5),
                        (state -> Logger.recordOutput("Drivetrain/Drive/SysIdTestState", state.toString()))),
                new SysIdRoutine.Mechanism(this::SysIdOnlyDriveMotorsForward, null, this));

        sysIdRoutineTurnMotors = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        Volts.of(6).per(Second),
                        Volts.of(2),
                        Seconds.of(1.0),
                        (state -> Logger.recordOutput("Drivetrain/Turn/SysIdTestState", state.toString()))),
                new SysIdRoutine.Mechanism(this::SysIdOnlyTurnMotors, null, this));
    }

    private DrivetrainControl getDrivetrainControl() {
        if (RobotState.isTeleop()) {
            return DashboardUI.Overview.getControl().getDrivetrainControl();
        } else {
            return DrivetrainControl.createRobotRelative(Transform2d.kZero, Transform2d.kZero, Transform2d.kZero);
        }
    }

    /** Drives the robot using robot relative ChassisSpeeds. */
    private void driveInternal() {
        DrivetrainControl drivetrainControl = getDrivetrainControl();

        int applyCount = 0;
        for (IDrivetrainControlModifier modifier : modifiers) {
            if (modifier.isEnabled()) {
                if (modifier.apply(drivetrainControl)) {
                    applyCount++;
                }
            }
        }

        lastModifiersAppliedCount = applyCount;

        ChassisSpeeds nonDiscreteSpeeds = drivetrainControl.toChassisSpeeds(); // Converts the control to ChassisSpeeds

        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint, // The previous setpoint
                nonDiscreteSpeeds, // The desired target speeds
                0.02 // The loop time of the robot code, in seconds
                );

        SwerveModuleState[] swerveModuleStates = previousSetpoint.moduleStates();

        // Sets state for each module
        if (SysIdManager.getSysIdRoutine() != SysIdManager.SysIdRoutine.DrivetrainSpin
                && SysIdManager.getSysIdRoutine() != SysIdManager.SysIdRoutine.DrivetrainForward) {
            m_frontLeft.setDesiredState(swerveModuleStates[0]);
            m_frontRight.setDesiredState(swerveModuleStates[1]);
            m_backLeft.setDesiredState(swerveModuleStates[2]);
            m_backRight.setDesiredState(swerveModuleStates[3]);
        }

        lastModuleSetpoints = swerveModuleStates;
    }

    @Override
    public void periodicManaged() {
        driveInternal();

        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_backLeft.periodic();
        m_backRight.periodic();
    }

    @Override
    public void simulationPeriodicManaged() {
        m_frontLeft.simulationPeriodic();
        m_frontRight.simulationPeriodic();
        m_backLeft.simulationPeriodic();
        m_backRight.simulationPeriodic();
    }

    public void SysIdOnlyDriveMotorsSpin(Voltage volts) {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(360 - 45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(180 + 45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(180 - 45)));

        m_frontLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        m_frontRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        m_backLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        m_backRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
    }

    public void SysIdOnlyDriveMotorsForward(Voltage volts) {

        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        m_frontLeft.setDesiredState(state);
        m_frontRight.setDesiredState(state);
        m_backLeft.setDesiredState(state);
        m_backRight.setDesiredState(state);

        m_frontLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        m_frontRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        m_backLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
        m_backRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
    }

    @AutoLogLevel(level = Level.Sysid)
    public double SysIdOnlyGetDriveMotorVolts() {
        // average of all drive motors .get() values
        return (m_frontLeft.getDriveMotorVoltsSysIdOnly()
                        + m_frontRight.getDriveMotorVoltsSysIdOnly()
                        + m_backLeft.getDriveMotorVoltsSysIdOnly()
                        + m_backRight.getDriveMotorVoltsSysIdOnly())
                / 4;
    }

    @AutoLogLevel(level = Level.Sysid)
    public double SysIdOnlyGetDriveMotorPosition() {
        // average
        return (m_frontLeft.getDriveWheelDistance()
                        + m_frontRight.getDriveWheelDistance()
                        + m_backLeft.getDriveWheelDistance()
                        + m_backRight.getDriveWheelDistance())
                / 4;
    }

    @AutoLogLevel(level = Level.Sysid)
    public double SysIdOnlyGetDriveMotorVelocity() {
        // average
        return (m_frontLeft.getDriveWheelVelocity()
                        + m_frontRight.getDriveWheelVelocity()
                        + m_backLeft.getDriveWheelVelocity()
                        + m_backRight.getDriveWheelVelocity())
                / 4;
    }

    public void SysIdOnlyTurnMotors(Voltage volts) {
        m_frontLeft.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
        m_frontRight.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
        m_backLeft.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
        m_backRight.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
    }

    @AutoLogLevel(level = Level.Sysid)
    public double SysIdOnlyGetTurnMotorVolts() {
        // average of all turn motors .get() values
        return (m_frontLeft.getTurnMotorVoltsSysIdOnly()
                        + m_frontRight.getTurnMotorVoltsSysIdOnly()
                        + m_backLeft.getTurnMotorVoltsSysIdOnly()
                        + m_backRight.getTurnMotorVoltsSysIdOnly())
                / 4;
    }

    @AutoLogLevel(level = Level.Sysid)
    public double SysIdOnlyGetTurnMotorPosition() {
        // average
        return (m_frontLeft.getTurnWheelRotation2d().getRotations()
                        + m_frontRight.getTurnWheelRotation2d().getRotations()
                        + m_backLeft.getTurnWheelRotation2d().getRotations()
                        + m_backRight.getTurnWheelRotation2d().getRotations())
                / 4;
    }

    @AutoLogLevel(level = Level.Sysid)
    public double SysIdOnlyGetTurnMotorVelocity() {
        // average
        return (m_frontLeft.getTurnWheelVelocity()
                        + m_frontRight.getTurnWheelVelocity()
                        + m_backLeft.getTurnWheelVelocity()
                        + m_backRight.getTurnWheelVelocity())
                / 4;
    }

    /**
     * Sets the PID target to zero and immediately stops all swerve modules.
     *
     * <p>This method commands the drivetrain to stop by setting the drive speeds to zero, thus
     * ensuring that the robot comes to a halt. It also directly stops each swerve module by setting
     * their motor outputs to zero.
     */
    @Override
    public void kill() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    /**
     * Retrieves the current chassis speeds relative to the robot's orientation.
     *
     * <p>This method calculates the chassis speeds based on the current states of all four swerve
     * modules using the drivetrain's kinematics.
     *
     * @return The current relative chassis speeds as a ChassisSpeeds object.
     */
    @AutoLogLevel(level = Level.Real)
    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(
                m_frontLeft.getModuleState(),
                m_frontRight.getModuleState(),
                m_backLeft.getModuleState(),
                m_backRight.getModuleState());
    }

    /**
     * Retrieves the current chassis acceleration relative to the robot's orientation.
     *
     * <p>This method calculates the chassis acceleration based on the current states of all four
     * swerve modules using the drivetrain's kinematics.
     *
     * @return The current relative chassis acceleration as a ChassisSpeeds object.
     */
    @AutoLogLevel(level = Level.Real)
    public ChassisSpeeds getChassisAcceleration() {
        return m_kinematics.toChassisSpeeds(
                m_frontLeft.getModuleStateAcceleration(),
                m_frontRight.getModuleStateAcceleration(),
                m_backLeft.getModuleStateAcceleration(),
                m_backRight.getModuleStateAcceleration());
    }

    @AutoLogLevel(level = Level.Real)
    public int getModifiersAppliedCount() {
        return lastModifiersAppliedCount;
    }

    /**
     * Returns the swerve drive kinematics for this drivetrain.
     *
     * @return The SwerveDriveKinematics object associated with this drivetrain.
     */
    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getModulePosition(),
            m_frontRight.getModulePosition(),
            m_backLeft.getModulePosition(),
            m_backRight.getModulePosition()
        };
    }

    @AutoLogLevel(level = Level.Real)
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getModuleState(),
            m_frontRight.getModuleState(),
            m_backLeft.getModuleState(),
            m_backRight.getModuleState()
        };
    }

    @AutoLogLevel(level = Level.Real)
    public SwerveModuleState[] getModuleSetpoints() {
        return lastModuleSetpoints;
    }

    public Command sysIdQuasistaticDriveMotorsSpin(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsSpin.quasistatic(direction);
    }

    public Command sysIdDynamicDriveMotorsSpin(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsSpin.dynamic(direction);
    }

    public Command sysIdQuasistaticDriveMotorsForward(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsForward.quasistatic(direction);
    }

    public Command sysIdDynamicDriveMotorsForward(SysIdRoutine.Direction direction) {
        return sysIdRoutineDriveMotorsForward.dynamic(direction);
    }

    public Command sysIdQuasistaticTurnMotors(SysIdRoutine.Direction direction) {
        return sysIdRoutineTurnMotors.quasistatic(direction);
    }

    public Command sysIdDynamicTurnMotors(SysIdRoutine.Direction direction) {
        return sysIdRoutineTurnMotors.dynamic(direction);
    }

    /** frees up all hardware allocations */
    public void close() throws Exception {
        m_backLeft.close();
        m_backRight.close();
        m_frontLeft.close();
        m_frontRight.close();
    }

    @Override
    public void setupShuffleboard() {
        SwerveModule[] modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
        for (SwerveModule module : modules) {
            module.setupShuffleboard();
        }
    }

    @Override
    public double getCurrentDrawAmps() {
        return m_frontLeft.getCurrentDrawAmps()
                + m_frontRight.getCurrentDrawAmps()
                + m_backLeft.getCurrentDrawAmps()
                + m_backRight.getCurrentDrawAmps();
    }
}
