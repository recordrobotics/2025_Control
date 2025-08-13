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
import edu.wpi.first.wpilibj2.command.DeferredCommand;
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
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.RepeatConditionallyCommand;
import frc.robot.utils.ShuffleboardPublisher;
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
public class RobotContainer {

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

    @SuppressWarnings("unused")
    private final AutoPath autoPath;

    private Command autoCommand;

    private Alert noEncoderResetAlert = new Alert("Encoders not reset!", AlertType.kError);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        if (Constants.RobotState.getMode() == Mode.REAL) {
            drivetrain = new Drivetrain();
            poseSensorFusion = new PoseSensorFusion();
            elevator = new Elevator(new ElevatorReal(Constants.Elevator.kDt));
            elevatorArm = new ElevatorArm(new ElevatorArmReal(0.02));
            elevatorHead = new ElevatorHead(new ElevatorHeadReal(0.02));
            coralIntake = new CoralIntake(new CoralIntakeReal(0.02));
            climber = new Climber(new ClimberReal(0.02));
            lights = new Lights();
            pdp = new PowerDistributionPanel();
            coralDetection = new CoralDetection();
        } else {
            if (Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.PHOTON_SIM) {
                visionSim = new VisionSystemSim("main");
                visionSim.addAprilTags(Constants.Game.APRILTAG_LAYOUT);
            }

            drivetrain = new Drivetrain();
            poseSensorFusion = new PoseSensorFusion();
            elevator = new Elevator(new ElevatorSim(Constants.Elevator.kDt));
            elevatorArm = new ElevatorArm(new ElevatorArmSim(0.02));
            elevatorHead = new ElevatorHead(new ElevatorHeadSim(0.02, drivetrain.getSwerveDriveSimulation()));
            coralIntake = new CoralIntake(new CoralIntakeSim(0.02, drivetrain.getSwerveDriveSimulation()));
            climber = new Climber(new ClimberSim(0.02));
            lights = new Lights();
            pdp = new PowerDistributionPanel();
            coralDetection = new CoralDetection();
            // uncomment to use an external photonvision client for coral detection simulation
            // coralDetection.setSimulationMode(CoralDetection.CoralDetectionSimulationMode.PHOTONVISION);
            humanPlayerSimulation = new HumanPlayerSimulation();
        }

        drivetrain.modifiers.add(new GroundIntakeAssist());

        model = new RobotModel();

        // Sets up auto path
        autoPath = new AutoPath();

        DashboardUI.Autonomous.setupAutoChooser();

        // Sets up Control scheme chooser
        DashboardUI.Overview.addControls(
                new JoystickXboxSimple(2, 0), new JoystickXbox(2, 0) /*, new XboxSimpleBackup(0)*/);

        configureLegacyTriggers();
        configureTriggers();

        ShuffleboardPublisher.setup(poseSensorFusion.nav, drivetrain, poseSensorFusion);

        elevator.setDefaultCommand(new ManualElevator());
        elevatorArm.setDefaultCommand(new ManualElevatorArm());

        noEncoderResetAlert.set(true);

        if (Constants.RobotState.getMode() != Mode.REAL) {
            // No point in manually resetting encoders in simulation since starting config is always in the right spot
            resetEncoders();
        }
    }

    public void teleopInit() {}

    public void disabledInit() {}

    /**
     * @deprecated This contains the old control scheme triggers
     */
    @Deprecated
    private void configureLegacyTriggers() {
        BooleanSupplier elevatorLock = () -> {
            boolean atBottom = elevator.getNearestHeight() == ElevatorHeight.INTAKE
                    || elevator.getNearestHeight() == ElevatorHeight.BOTTOM;
            boolean atL4 = elevator.getNearestHeight() == ElevatorHeight.L4
                    || elevator.getNearestHeight() == ElevatorHeight.BARGE_ALAGAE;

            Pose2d robot = RobotContainer.poseSensorFusion.getEstimatedPosition();
            CoralPosition closestReef = IGamePosition.closestTo(robot, CoralPosition.values());

            boolean nearReef =
                    closestReef.getFirstStagePose().getTranslation().getDistance(robot.getTranslation()) < 0.2
                            && Math.abs(closestReef
                                            .getFirstStagePose()
                                            .getRotation()
                                            .minus(robot.getRotation())
                                            .getDegrees())
                                    < 80;

            return DashboardUI.Overview.getControl().getManualOverride()
                    || (atBottom && elevatorHead.coralReady())
                    || (atL4 && !nearReef)
                    || (!atBottom && !atL4);
        };

        new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL2() && elevatorLock.getAsBoolean())
                .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L2));
        new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL3() && elevatorLock.getAsBoolean())
                .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L3));
        new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL4() && elevatorLock.getAsBoolean())
                .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L4));

        new Trigger(() -> (DashboardUI.Overview.getControl().getElevatorL2()
                                || DashboardUI.Overview.getControl().getElevatorL3()
                                || DashboardUI.Overview.getControl().getElevatorL4())
                        && !elevatorLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));

        new Trigger(() -> DashboardUI.Overview.getControl().getCoralShoot()).onTrue(new CoralShoot());
        new Trigger(() -> DashboardUI.Overview.getControl().getCoralGroundIntake())
                .toggleOnTrue(new CoralIntakeFromGroundToggled());

        new Trigger(() -> DashboardUI.Overview.getControl().getCoralSourceIntake())
                .onTrue(new CoralIntakeFromSource(true));

        Command coralScoreL1Cmd = Commands.either(
                Commands.either(
                        new CoralIntakeShootL1().asProxy(),
                        new CoralIntakeMoveL1().asProxy(),
                        () -> coralIntake.getState() == CoralIntakeState.L1_DOWN),
                new CoralIntakeFromGroundUpL1()
                        .asProxy()
                        .beforeStarting(() -> CoralIntakeFromGroundToggled.isGoingToL1 = true),
                () -> coralIntake.getState() != CoralIntakeState.GROUND);
        // coralScoreL1Cmd.addRequirements(coralIntakeMoveToggleRequirement);

        new Trigger(() -> DashboardUI.Overview.getControl().getCoralIntakeScoreL1()).onTrue(coralScoreL1Cmd.asProxy());

        BooleanSupplier algaeLock = () -> DashboardUI.Overview.getControl().getManualOverride()
                || !elevatorHead.getGamePiece().atLeast(GamePiece.CORAL);

        new Trigger(() -> DashboardUI.Overview.getControl().getGroundAlgae() && algaeLock.getAsBoolean())
                .toggleOnTrue(new GroundAlgaeToggled(ElevatorHeight.GROUND_ALGAE));
        new Trigger(() -> DashboardUI.Overview.getControl().getElevatorAlgaeLow() && algaeLock.getAsBoolean())
                .toggleOnTrue(new ElevatorAlgaeToggled(ElevatorHeight.LOW_REEF_ALGAE));
        new Trigger(() -> DashboardUI.Overview.getControl().getElevatorAlgaeHigh() && algaeLock.getAsBoolean())
                .toggleOnTrue(new ElevatorAlgaeToggled(ElevatorHeight.HIGH_REEF_ALGAE));
        new Trigger(() -> DashboardUI.Overview.getControl().getScoreAlgae() && algaeLock.getAsBoolean())
                .onTrue(new ProcessorScore(true));

        new Trigger(() -> (DashboardUI.Overview.getControl().getGroundAlgae()
                                || DashboardUI.Overview.getControl().getElevatorAlgaeLow())
                        && !algaeLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));
        new Trigger(() -> DashboardUI.Overview.getControl().getElevatorAlgaeHigh() && !algaeLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kBothRumble, 1).withTimeout(0.1));
        new Trigger(() -> DashboardUI.Overview.getControl().getScoreAlgae() && !algaeLock.getAsBoolean())
                .onTrue(new VibrateXbox(RumbleType.kLeftRumble, 1).withTimeout(0.1));

        new Trigger(() -> DashboardUI.Overview.getControl().getAutoAlign())
                .whileTrue(WaypointAlign.align(
                        ReefAlign.generateWaypointsClosest(false), new Boolean[] {true, true}, new Double[] {2.0, 1.0
                        }));
    }

    private void configureTriggers() {

        // Command to kill robot
        new Trigger(() -> DashboardUI.Overview.getControl().getKill())
                .whileTrue(new KillSpecified(drivetrain, elevator, elevatorArm, elevatorHead, coralIntake, climber)
                        .alongWith(new InstantCommand(
                                () -> CommandScheduler.getInstance().cancelAll())));

        new Trigger(() -> DashboardUI.Overview.getControl().getClimb())
                .onTrue(Commands.either(
                                new ClimbMove(ClimberState.Climb),
                                new ClimbMove(ClimberState.Extend),
                                () -> climber.getCurrentState() == ClimberState.Extend)
                        .alongWith(new InstantCommand(() -> Elastic.selectTab("Climb"))));

        // Reset pose trigger
        new Trigger(() -> DashboardUI.Overview.getControl().getPoseReset())
                .onTrue(new InstantCommand(poseSensorFusion::resetDriverPose));
        new Trigger(() -> DashboardUI.Overview.getControl().getLimelightReset())
                .onTrue(new InstantCommand(poseSensorFusion::resetToVision));
        new Trigger(() -> DashboardUI.Autonomous.getResetLocation())
                .onTrue(new InstantCommand(poseSensorFusion::resetStartingPose).ignoringDisable(true));
        new Trigger(() -> DashboardUI.Autonomous.getLimelightRotation())
                .onTrue(new InstantCommand(poseSensorFusion::resetToVision).ignoringDisable(true));
        new Trigger(() -> DashboardUI.Autonomous.getEncoderReset())
                .onTrue(new InstantCommand(this::resetEncoders).ignoringDisable(true));

        new Trigger(() -> DashboardUI.Overview.getControl().getCoralGroundIntakeSimple()
                        && !elevatorHead.getGamePiece().atLeast(GamePiece.CORAL_CERTAIN))
                .onTrue(new CoralIntakeFromGround())
                .onFalse(Commands.either(
                                new CoralIntakeFromGroundUpL1(),
                                new CoralIntakeFromGroundUpSimple()
                                        .handleInterrupt(() -> {
                                            CoralIntakeFromGroundUpSimple.isRunning = false;
                                        })
                                        .andThen(new CoralIntakeSimple(false)
                                                .finallyDo(() -> {
                                                    CoralIntakeSimple.isRunning = false;
                                                    CoralIntakeFromGroundUpSimple.isRunning = false;
                                                })
                                                .onlyIf(() -> !CoralIntakeSimple.isRunning)),
                                () -> DashboardUI.Overview.getControl().getReefLevelSwitchValue()
                                        == ReefLevelSwitchValue.L1)
                        .onlyWhile(() -> elevatorHead.getGamePiece().atLeast(GamePiece.CORAL)
                                || !DashboardUI.Overview.getControl().getCoralGroundIntakeSimple()));

        new Trigger(() -> DashboardUI.Overview.getControl().getCoralSourceIntakeAuto() && !DriverStation.isAutonomous())
                .debounce(1.0, DebounceType.kFalling)
                .whileTrue(new RepeatConditionallyCommand(
                                new CoralIntakeSimple(true)
                                        .finallyDo(() -> {
                                            CoralIntakeSimple.isRunning = false;
                                            RobotContainer.elevatorHead.set(CoralShooterStates.OFF);
                                            RobotContainer.coralIntake.set(CoralIntakeState.UP);
                                        })
                                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                                        .asProxy()
                                        .onlyIf(() -> !CoralIntakeSimple.isRunning
                                                && !CoralIntakeFromGroundUpSimple.isRunning
                                                && !AutoAlgae.isRunning()),
                                () -> !RobotContainer.elevatorHead
                                                .getGamePiece()
                                                .atLeast(GamePiece.CORAL_CERTAIN)
                                        && !DashboardUI.Overview.getControl().getCoralGroundIntakeSimple()
                                        && DashboardUI.Overview.getControl().getReefLevelSwitchValue()
                                                != ReefLevelSwitchValue.L1,
                                false)
                        .ignoringDisable(true));

        new Trigger(() -> DashboardUI.Overview.getControl().getReefAlgaeSimple()
                        && !elevatorHead.getGamePiece().atLeast(GamePiece.CORAL_CERTAIN))
                .onTrue(Commands.either(
                        new DeferredCommand(
                                () -> new ScheduleCommand(new AutoAlgae(IGamePosition.closestTo(
                                                RobotContainer.poseSensorFusion.getEstimatedPosition(),
                                                AlgaePosition.values()))
                                        .finallyDo(() -> AutoAlgae.stopRunning())
                                        .asProxy()),
                                Set.of()),
                        new InstantCommand(() -> AutoAlgae.performCancel()),
                        () -> !AutoAlgae.isRunning()));

        new Trigger(() -> DashboardUI.Overview.getControl().getAutoScore())
                .onTrue(Commands.either(
                        new DeferredCommand(
                                () -> new AutoScore(IGamePosition.closestTo(
                                                RobotContainer.poseSensorFusion.getEstimatedPosition(),
                                                CoralPosition.values()))
                                        .asProxy(),
                                Set.of()),
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

        if (SysIdManager.getSysIdRoutine() != SysIdRoutine.None) {
            return SysIdManager.getSysIdRoutine().createCommand();
        }

        if (autoCommand == null) {
            autoCommand = new PlannedAuto();
        }
        return autoCommand;
    }

    public void testPeriodic() {
        DashboardUI.Test.testPeriodic();
    }

    public void simulationPeriodic() {
        updateSimulationBattery(drivetrain, elevator, elevatorHead, coralIntake);
        if (Constants.RobotState.VISION_SIMULATION_MODE == VisionSimulationMode.PHOTON_SIM) {
            visionSim.update(model.getRobot());
        }
    }

    public void updateSimulationBattery(PoweredSubsystem... subsystems) {
        double[] currents = new double[subsystems.length];
        for (int i = 0; i < subsystems.length; i++) {
            currents[i] = subsystems[i].getCurrentDrawAmps();
        }

        // RoboRioSim.setVInVoltage(
        //     MathUtil.clamp(BatterySim.calculateDefaultBatteryLoadedVoltage(currents), 0, 13));
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
    public void close() throws Exception {
        drivetrain.close();
        poseSensorFusion.close();
        elevator.close();
        elevatorArm.close();
        elevatorHead.close();
        coralIntake.close();
        pdp.close();
    }
}
