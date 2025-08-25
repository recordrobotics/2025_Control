package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
// WPILib imports
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.AlgaePosition;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.Constants.RobotState.VisionSimulationMode;
import frc.robot.commands.AutoAlgae;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ClimbMove;
import frc.robot.commands.CoralIntakeFromGround;
import frc.robot.commands.CoralIntakeFromGroundUpL1;
import frc.robot.commands.CoralIntakeFromGroundUpSimple;
import frc.robot.commands.CoralIntakeMoveL1;
import frc.robot.commands.CoralIntakeShootL1;
import frc.robot.commands.CoralIntakeSimple;
import frc.robot.commands.CoralShoot;
// Local imports
import frc.robot.commands.KillSpecified;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.VibrateXbox;
import frc.robot.commands.WaypointAlign;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.commands.legacy.CoralIntakeFromGroundToggled;
import frc.robot.commands.legacy.CoralIntakeFromSource;
import frc.robot.commands.legacy.ElevatorAlgaeToggled;
import frc.robot.commands.legacy.ElevatorReefToggled;
import frc.robot.commands.legacy.GroundAlgaeToggled;
import frc.robot.commands.manual.ManualElevator;
import frc.robot.commands.manual.ManualElevatorArm;
import frc.robot.control.*;
import frc.robot.control.AbstractControl.ReefLevelSwitchValue;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
import frc.robot.subsystems.ElevatorHead.GamePiece;
import frc.robot.subsystems.io.real.ClimberReal;
import frc.robot.subsystems.io.real.CoralIntakeReal;
import frc.robot.subsystems.io.real.ElevatorArmReal;
import frc.robot.subsystems.io.real.ElevatorHeadReal;
import frc.robot.subsystems.io.real.ElevatorReal;
import frc.robot.subsystems.io.sim.ClimberSim;
import frc.robot.subsystems.io.sim.CoralIntakeSim;
import frc.robot.subsystems.io.sim.ElevatorArmSim;
import frc.robot.subsystems.io.sim.ElevatorHeadSim;
import frc.robot.subsystems.io.sim.ElevatorSim;
import frc.robot.utils.AutoPath;
import frc.robot.utils.HumanPlayerSimulation;
import frc.robot.utils.ModuleConstants.InvalidConfigException;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.RepeatConditionallyCommand;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdRoutine;
import frc.robot.utils.libraries.Elastic;
import frc.robot.utils.libraries.Elastic.Notification;
import frc.robot.utils.libraries.Elastic.Notification.NotificationLevel;
import frc.robot.utils.modifiers.GroundIntakeAssist;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.photonvision.simulation.VisionSystemSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings({"java:S1444", "java:S1104", "java:S3010"})
public final class RobotContainer {

    public static Drivetrain drivetrain;
    public static PoseSensorFusion poseSensorFusion;
    public static Elevator elevator;
    public static ElevatorArm elevatorArm;
    public static ElevatorHead elevatorHead;
    public static Climber climber;
    public static Lights lights;
    public static PowerDistributionPanel pdp;
    public static CoralIntake coralIntake;
    public static RobotModel model;
    public static CoralDetection coralDetection;
    public static VisionSystemSim visionSim;
    public static HumanPlayerSimulation humanPlayerSimulation;

    private Command autoCommand;

    private Alert noEncoderResetAlert = new Alert("Encoders not reset!", AlertType.kError);

    public static final double ROBOT_PERIODIC = 0.02;
    public static final int CONTROL_JOYSTICK_PORT = 2;
    public static final int CONTROL_XBOX_PORT = 0;

    public static final double INVALID_COMMAND_VIBRATE_TIME = 0.1;

    public static final double ELEVATOR_LOCK_REEF_DISTANCE = 0.2;
    public static final double ELEVATOR_LOCK_REEF_ANGLE_DIFF = 80; /* degrees */

    public static final double AUTO_ALIGN_FIRST_WAYPOINT_TIMEOUT = 2.0;
    public static final double AUTO_ALIGN_SECOND_WAYPOINT_TIMEOUT = 1.0;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        try {
            drivetrain = new Drivetrain();
        } catch (InvalidConfigException e) {
            e.printStackTrace();
            DriverStation.reportError("Could not load drivetrain config!: " + e.getMessage(), false);
            return;
        }

        if (Constants.RobotState.getMode() == Mode.REAL) {
            poseSensorFusion = new PoseSensorFusion();
            elevator = new Elevator(new ElevatorReal(ROBOT_PERIODIC));
            elevatorArm = new ElevatorArm(new ElevatorArmReal(ROBOT_PERIODIC));
            elevatorHead = new ElevatorHead(new ElevatorHeadReal(ROBOT_PERIODIC));
            coralIntake = new CoralIntake(new CoralIntakeReal(ROBOT_PERIODIC));
            climber = new Climber(new ClimberReal(ROBOT_PERIODIC));
            lights = new Lights();
            pdp = new PowerDistributionPanel();
            coralDetection = new CoralDetection();
        } else {
            if (Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.PHOTON_SIM) {
                visionSim = new VisionSystemSim("main");
                visionSim.addAprilTags(Constants.Game.APRILTAG_LAYOUT);
            }

            poseSensorFusion = new PoseSensorFusion();
            elevator = new Elevator(new ElevatorSim(ROBOT_PERIODIC));
            elevatorArm = new ElevatorArm(new ElevatorArmSim(ROBOT_PERIODIC));
            elevatorHead = new ElevatorHead(new ElevatorHeadSim(ROBOT_PERIODIC, drivetrain.getSwerveDriveSimulation()));
            coralIntake = new CoralIntake(new CoralIntakeSim(ROBOT_PERIODIC, drivetrain.getSwerveDriveSimulation()));
            climber = new Climber(new ClimberSim(ROBOT_PERIODIC));
            lights = new Lights();
            pdp = new PowerDistributionPanel();
            coralDetection = new CoralDetection();
            humanPlayerSimulation = new HumanPlayerSimulation();
        }

        drivetrain.modifiers.add(new GroundIntakeAssist());

        model = new RobotModel();

        // Sets up auto path
        AutoPath.initialize();

        DashboardUI.Autonomous.setupAutoChooser();

        // Sets up Control scheme chooser
        DashboardUI.Overview.addControls(
                new JoystickXboxSimple(CONTROL_JOYSTICK_PORT, CONTROL_XBOX_PORT),
                new JoystickXbox(
                        CONTROL_JOYSTICK_PORT, CONTROL_XBOX_PORT) /*, new XboxSimpleBackup(CONTROL_XBOX_PORT)*/);

        configureLegacyTriggers();
        configureTriggers();

        elevator.setDefaultCommand(new ManualElevator());
        elevatorArm.setDefaultCommand(new ManualElevatorArm());

        noEncoderResetAlert.set(true);

        if (Constants.RobotState.getMode() != Mode.REAL) {
            // No point in manually resetting encoders in simulation since starting config is always in the right spot
            resetEncoders();
        }
    }

    public void teleopInit() {
        /* nothing to do */
    }

    public void disabledInit() {
        /* nothing to do */
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureLegacyTriggers() {
        BooleanSupplier elevatorLock = createElevatorLockSupplier();
        BooleanSupplier algaeLock = createAlgaeLockSupplier();

        configureElevatorTriggers(elevatorLock);
        configureCoralTriggers();
        configureAlgaeTriggers(algaeLock);
        configureAutoAlignTrigger();
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static BooleanSupplier createElevatorLockSupplier() {
        return () -> {
            boolean atBottom = elevator.getNearestHeight() == ElevatorHeight.INTAKE
                    || elevator.getNearestHeight() == ElevatorHeight.BOTTOM;
            boolean atL4 = elevator.getNearestHeight() == ElevatorHeight.L4
                    || elevator.getNearestHeight() == ElevatorHeight.BARGE_ALGAE;

            Pose2d robot = RobotContainer.poseSensorFusion.getEstimatedPosition();
            CoralPosition closestReef = IGamePosition.closestTo(robot, CoralPosition.values());

            boolean nearReef = isNearReef(robot, closestReef);

            return DashboardUI.Overview.getControl().isManualOverrideTriggered()
                    || (atBottom && elevatorHead.coralReady())
                    || (atL4 && !nearReef)
                    || (!atBottom && !atL4);
        };
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static boolean isNearReef(Pose2d robot, CoralPosition closestReef) {
        return closestReef.getFirstStagePose().getTranslation().getDistance(robot.getTranslation())
                        < ELEVATOR_LOCK_REEF_DISTANCE
                && Math.abs(closestReef
                                .getFirstStagePose()
                                .getRotation()
                                .minus(robot.getRotation())
                                .getDegrees())
                        < ELEVATOR_LOCK_REEF_ANGLE_DIFF;
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static BooleanSupplier createAlgaeLockSupplier() {
        return () -> DashboardUI.Overview.getControl().isManualOverrideTriggered()
                || !elevatorHead.getGamePiece().atLeast(GamePiece.CORAL);
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureElevatorTriggers(BooleanSupplier elevatorLock) {
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorL2Triggered() && elevatorLock.getAsBoolean())
                .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L2));
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorL3Triggered() && elevatorLock.getAsBoolean())
                .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L3));
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorL4Triggered() && elevatorLock.getAsBoolean())
                .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L4));

        new Trigger(() -> (DashboardUI.Overview.getControl().isElevatorL2Triggered()
                                || DashboardUI.Overview.getControl().isElevatorL3Triggered()
                                || DashboardUI.Overview.getControl().isElevatorL4Triggered())
                        && !elevatorLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(INVALID_COMMAND_VIBRATE_TIME));
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureCoralTriggers() {
        new Trigger(() -> DashboardUI.Overview.getControl().isCoralShootTriggered()).onTrue(new CoralShoot());
        new Trigger(() -> DashboardUI.Overview.getControl().isCoralGroundIntakeTriggered())
                .toggleOnTrue(new CoralIntakeFromGroundToggled());

        new Trigger(() -> DashboardUI.Overview.getControl().isCoralSourceIntakeTriggered())
                .onTrue(new CoralIntakeFromSource(true));

        Command coralScoreL1Cmd = createCoralScoreL1Command();
        new Trigger(() -> DashboardUI.Overview.getControl().isCoralIntakeScoreL1Triggered())
                .onTrue(coralScoreL1Cmd.asProxy());
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static Command createCoralScoreL1Command() {
        return Commands.either(
                Commands.either(
                        new CoralIntakeShootL1().asProxy(),
                        new CoralIntakeMoveL1().asProxy(),
                        () -> coralIntake.getState() == CoralIntakeState.L1_DOWN),
                new CoralIntakeFromGroundUpL1()
                        .asProxy()
                        .beforeStarting(() -> CoralIntakeFromGroundToggled.isGoingToL1 = true),
                () -> coralIntake.getState() != CoralIntakeState.GROUND);
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureAlgaeTriggers(BooleanSupplier algaeLock) {
        new Trigger(() -> DashboardUI.Overview.getControl().isGroundAlgaeTriggered() && algaeLock.getAsBoolean())
                .toggleOnTrue(new GroundAlgaeToggled(ElevatorHeight.GROUND_ALGAE));
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorAlgaeLowTriggered() && algaeLock.getAsBoolean())
                .toggleOnTrue(new ElevatorAlgaeToggled(ElevatorHeight.LOW_REEF_ALGAE));
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorAlgaeHighTriggered() && algaeLock.getAsBoolean())
                .toggleOnTrue(new ElevatorAlgaeToggled(ElevatorHeight.HIGH_REEF_ALGAE));
        new Trigger(() -> DashboardUI.Overview.getControl().isScoreAlgaeTriggered() && algaeLock.getAsBoolean())
                .onTrue(new ProcessorScore(true));

        configureAlgaeVibrateXboxTriggers(algaeLock);
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureAlgaeVibrateXboxTriggers(BooleanSupplier algaeLock) {
        new Trigger(() -> (DashboardUI.Overview.getControl().isGroundAlgaeTriggered()
                                || DashboardUI.Overview.getControl().isElevatorAlgaeLowTriggered())
                        && !algaeLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(INVALID_COMMAND_VIBRATE_TIME));
        new Trigger(() -> DashboardUI.Overview.getControl().isElevatorAlgaeHighTriggered() && !algaeLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kBothRumble, 1).withTimeout(INVALID_COMMAND_VIBRATE_TIME));
        new Trigger(() -> DashboardUI.Overview.getControl().isScoreAlgaeTriggered() && !algaeLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kLeftRumble, 1).withTimeout(INVALID_COMMAND_VIBRATE_TIME));
    }

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated(forRemoval = true)
    private static void configureAutoAlignTrigger() {
        new Trigger(() -> DashboardUI.Overview.getControl().isAutoAlignTriggered())
                .whileTrue(Commands.defer(
                        () -> WaypointAlign.align(ReefAlign.generateWaypointsClosest(false), 0, 1, true, new Double[] {
                            AUTO_ALIGN_FIRST_WAYPOINT_TIMEOUT, AUTO_ALIGN_SECOND_WAYPOINT_TIMEOUT
                        }),
                        Set.of(drivetrain)));
    }

    private void configureTriggers() {

        // Command to kill robot
        new Trigger(() -> DashboardUI.Overview.getControl().isKillTriggered())
                .whileTrue(new KillSpecified(drivetrain, elevatorHead, coralIntake, climber)
                        .alongWith(new InstantCommand(
                                () -> CommandScheduler.getInstance().cancelAll())));

        new Trigger(() -> DashboardUI.Overview.getControl().isClimbTriggered())
                .onTrue(Commands.either(
                                new ClimbMove(ClimberState.CLIMB),
                                new ClimbMove(ClimberState.EXTEND),
                                () -> climber.getCurrentState() == ClimberState.EXTEND)
                        .alongWith(new InstantCommand(() -> Elastic.selectTab("Climb"))));

        // Reset pose trigger
        new Trigger(() -> DashboardUI.Overview.getControl().isPoseResetTriggered())
                .onTrue(new InstantCommand(poseSensorFusion::resetDriverPose));
        new Trigger(() -> DashboardUI.Overview.getControl().isLimelightResetTriggered())
                .onTrue(new InstantCommand(poseSensorFusion::resetToVision));
        new Trigger(DashboardUI.Autonomous::isResetLocationPressed)
                .onTrue(new InstantCommand(poseSensorFusion::resetStartingPose).ignoringDisable(true));
        new Trigger(DashboardUI.Autonomous::isLimelightRotationPressed)
                .onTrue(new InstantCommand(poseSensorFusion::resetToVision).ignoringDisable(true));
        new Trigger(DashboardUI.Autonomous::isEncoderResetPressed)
                .onTrue(new InstantCommand(this::resetEncoders).ignoringDisable(true));

        new Trigger(() -> DashboardUI.Overview.getControl().isCoralGroundIntakeSimpleTriggered()
                        && !elevatorHead.getGamePiece().atLeast(GamePiece.CORAL_CERTAIN))
                .onTrue(new CoralIntakeFromGround())
                .onFalse(Commands.either(
                                new CoralIntakeFromGroundUpL1(),
                                new CoralIntakeFromGroundUpSimple()
                                        .handleInterrupt(() -> CoralIntakeFromGroundUpSimple.setRunning(false))
                                        .andThen(new CoralIntakeSimple(false)
                                                .finallyDo(() -> {
                                                    CoralIntakeSimple.setRunning(false);
                                                    CoralIntakeFromGroundUpSimple.setRunning(false);
                                                })
                                                .onlyIf(() -> !CoralIntakeSimple.isRunning())),
                                () -> DashboardUI.Overview.getControl().getReefLevelSwitchValue()
                                        == ReefLevelSwitchValue.L1)
                        .onlyWhile(() -> elevatorHead.getGamePiece().atLeast(GamePiece.CORAL)
                                || !DashboardUI.Overview.getControl().isCoralGroundIntakeSimpleTriggered()));

        new Trigger(() -> DashboardUI.Overview.getControl().isCoralSourceIntakeAutoTriggered()
                        && !DriverStation.isAutonomous())
                .debounce(1.0, DebounceType.kFalling)
                .whileTrue(new RepeatConditionallyCommand(
                                new CoralIntakeSimple(true)
                                        .finallyDo(() -> {
                                            CoralIntakeSimple.setRunning(false);
                                            RobotContainer.elevatorHead.set(CoralShooterStates.OFF);
                                            RobotContainer.coralIntake.set(CoralIntakeState.UP);
                                        })
                                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                                        .asProxy()
                                        .onlyIf(() -> !CoralIntakeSimple.isRunning()
                                                && !CoralIntakeFromGroundUpSimple.isRunning()
                                                && !AutoAlgae.isRunning()),
                                () -> !RobotContainer.elevatorHead
                                                .getGamePiece()
                                                .atLeast(GamePiece.CORAL_CERTAIN)
                                        && !DashboardUI.Overview.getControl().isCoralGroundIntakeSimpleTriggered()
                                        && DashboardUI.Overview.getControl().getReefLevelSwitchValue()
                                                != ReefLevelSwitchValue.L1,
                                false)
                        .ignoringDisable(true));

        new Trigger(() -> DashboardUI.Overview.getControl().isReefAlgaeSimpleTriggered()
                        && !elevatorHead.getGamePiece().atLeast(GamePiece.CORAL_CERTAIN))
                .onTrue(Commands.either(
                        Commands.defer(
                                () -> new ScheduleCommand(new AutoAlgae(IGamePosition.closestTo(
                                                RobotContainer.poseSensorFusion.getEstimatedPosition(),
                                                AlgaePosition.values()))
                                        .finallyDo(AutoAlgae::stopRunning)
                                        .asProxy()),
                                Set.of()),
                        new InstantCommand(AutoAlgae::performCancel),
                        () -> !AutoAlgae.isRunning()));

        new Trigger(() -> DashboardUI.Overview.getControl().isAutoScoreTriggered())
                .onTrue(Commands.either(
                        Commands.deferredProxy(() -> new AutoScore(
                                IGamePosition.closestTo(
                                        RobotContainer.poseSensorFusion.getEstimatedPosition(), CoralPosition.values()),
                                DashboardUI.Overview.getControl()
                                        .getReefLevelSwitchValue()
                                        .toCoralLevel())),
                        new ProcessorScore(false).asProxy(),
                        () -> elevatorHead.getGamePiece().atLeast(GamePiece.CORAL)
                                || (DashboardUI.Overview.getControl().getReefLevelSwitchValue()
                                                == ReefLevelSwitchValue.L1
                                        && !elevatorHead.getGamePiece().atLeast(GamePiece.ALGAE))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        if (SysIdManager.getSysIdRoutine() != SysIdRoutine.NONE) {
            return SysIdManager.getSysIdRoutine().createCommand();
        }

        if (autoCommand == null) {
            autoCommand = new PlannedAuto();
        }
        return autoCommand;
    }

    public void simulationPeriodic() {
        updateSimulationBattery(drivetrain, elevator, elevatorHead, coralIntake);
        if (Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.PHOTON_SIM) {
            visionSim.update(model.getRobot());
        }
    }

    @SuppressWarnings("java:S2325")
    public void updateSimulationBattery(PoweredSubsystem... subsystems) {
        double[] currents = new double[subsystems.length];
        for (int i = 0; i < subsystems.length; i++) {
            currents[i] = subsystems[i].getCurrentDrawAmps();
        }
    }

    public void resetEncoders() {
        climber.resetEncoders();
        elevator.resetEncoders();
        elevatorArm.resetEncoders();
        coralIntake.resetEncoders();

        noEncoderResetAlert.set(false);
        Elastic.sendNotification(
                new Notification(NotificationLevel.INFO, "Encoders reset!", "Successfully reset arm encoders."));
    }

    /** frees up all hardware allocations */
    public static void close() throws Exception {
        drivetrain.close();
        poseSensorFusion.close();
        elevator.close();
        elevatorArm.close();
        elevatorHead.close();
        coralIntake.close();
        pdp.close();
    }
}
