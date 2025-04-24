package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
// WPILib imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ClimbMove;
import frc.robot.commands.CoralIntakeFromGroundToggled;
import frc.robot.commands.CoralIntakeFromGroundUpL1;
import frc.robot.commands.CoralIntakeFromSource;
import frc.robot.commands.CoralIntakeMoveL1;
import frc.robot.commands.CoralIntakeShootL1;
import frc.robot.commands.CoralShoot;
import frc.robot.commands.ElevatorAlgaeToggled;
import frc.robot.commands.ElevatorReefToggled;
import frc.robot.commands.GroundAlgaeToggled;
// Local imports
import frc.robot.commands.KillSpecified;
import frc.robot.commands.ProcessorScore;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.VibrateXbox;
import frc.robot.commands.manual.ManualElevator;
import frc.robot.commands.manual.ManualElevatorArm;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.control.*;
import frc.robot.control.AbstractControl.AutoScoreDirection;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.io.real.ClimberReal;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
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
import frc.robot.utils.assists.GroundIntakeAssist;
import frc.robot.utils.assists.IAssist;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.simulation.VisionSystemSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final RobotModel model = new RobotModel();

  public static Drivetrain drivetrain;
  public static PoseSensorFusion poseSensorFusion;
  public static Elevator elevator;
  public static ElevatorArm elevatorArm;
  public static ElevatorHead elevatorHead;
  public static CoralIntake coralIntake;
  public static Climber climber;
  public static Lights lights;
  public static PowerDistributionPanel pdp;

  public static CoralDetection coralDetection;

  public static VisionSystemSim visionSim;

  public static HumanPlayerSimulation humanPlayerSimulation;

  // Autonomous
  @SuppressWarnings("unused")
  private final AutoPath autoPath;

  private Command autoCommand;

  public static class ElevatorMoveToggleRequirement extends SubsystemBase {}

  public static ElevatorMoveToggleRequirement elevatorMoveToggleRequirement =
      new ElevatorMoveToggleRequirement();

  public static class CoralIntakeMoveToggleRequirement extends SubsystemBase {}

  public static CoralIntakeMoveToggleRequirement coralIntakeMoveToggleRequirement =
      new CoralIntakeMoveToggleRequirement();

  @AutoLogOutput private static boolean inClimbMode = false;

  public static final List<IAssist> assits = List.of(new GroundIntakeAssist());

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
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(Constants.Game.APRILTAG_LAYOUT);

      drivetrain = new Drivetrain();
      poseSensorFusion = new PoseSensorFusion();
      elevator = new Elevator(new ElevatorSim(Constants.Elevator.kDt));
      elevatorArm = new ElevatorArm(new ElevatorArmSim(0.02));
      elevatorHead =
          new ElevatorHead(new ElevatorHeadSim(0.02, drivetrain.getSwerveDriveSimulation()));
      coralIntake =
          new CoralIntake(new CoralIntakeSim(0.02, drivetrain.getSwerveDriveSimulation()));
      climber = new Climber(new ClimberSim(0.02));
      lights = new Lights();
      pdp = new PowerDistributionPanel();
      coralDetection = new CoralDetection();
      // uncomment to use an external photonvision client for coral detection simulation
      // coralDetection.setSimulationMode(CoralDetection.CoralDetectionSimulationMode.PHOTONVISION);
      humanPlayerSimulation = new HumanPlayerSimulation();
    }

    // Sets up auto path
    autoPath = new AutoPath();

    DashboardUI.Autonomous.setupAutoChooser();

    // Sets up Control scheme chooser
    DashboardUI.Overview.addControls(new JoystickXbox(2, 0), new JoystickXboxSimple(2, 0));

    // Bindings and Teleop
    configureButtonBindings();

    ShuffleboardPublisher.setup(poseSensorFusion.nav, drivetrain, poseSensorFusion);

    drivetrain.setDefaultCommand(new ManualSwerve());
    elevator.setDefaultCommand(new ManualElevator());
    elevatorArm.setDefaultCommand(new ManualElevatorArm());
  }

  public void teleopInit() {
    inClimbMode = false;
  }

  public void disabledInit() {
    inClimbMode = false;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Command to kill robot
    new Trigger(() -> DashboardUI.Overview.getControl().getKill())
        .whileTrue(
            new KillSpecified(drivetrain, elevator, elevatorArm, elevatorHead, coralIntake, climber)
                .alongWith(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())));

    new Trigger(() -> DashboardUI.Overview.getControl().getClimb())
        .onTrue(
            Commands.either(
                new ClimbMove(ClimberState.Climb),
                new ClimbMove(ClimberState.Extend),
                () -> climber.getCurrentState() == ClimberState.Extend));

    // Reset pose trigger
    new Trigger(() -> DashboardUI.Overview.getControl().getPoseReset())
        .onTrue(new InstantCommand(poseSensorFusion::resetDriverPose));
    new Trigger(() -> DashboardUI.Overview.getControl().getLimelightReset())
        .onTrue(new InstantCommand(poseSensorFusion::resetFullLimelight));
    new Trigger(() -> DashboardUI.Autonomous.getResetLocation())
        .onTrue(new InstantCommand(poseSensorFusion::resetStartingPose).ignoringDisable(true));
    new Trigger(() -> DashboardUI.Autonomous.getLimelightRotation())
        .onTrue(new InstantCommand(poseSensorFusion::resetToLimelight).ignoringDisable(true));

    // Climb mode trigger
    // new Trigger(() -> DashboardUI.Overview.getControl().getClimbMode())
    //     .onTrue(
    //         new InstantCommand(
    //                 () -> {
    //                   if (!inClimbMode) {
    //                     inClimbMode = true;
    //                     Elastic.selectTab("Climb");
    //                   } else {
    //                     inClimbMode = false;
    //                     DashboardUI.Overview.switchTo();
    //                   }
    //                 })
    //             .andThen(
    //                 new DeferredCommand(
    //                     () ->
    //                         new ClimbMove(
    //                             climber.getCurrentState() == ClimberState.Extend
    //                                 ? ClimberState.Park
    //                                 : ClimberState.Extend),
    //                     Set.of(climber))));

    BooleanSupplier elevatorLock =
        () -> {
          boolean atBottom =
              elevator.getNearestHeight() == ElevatorHeight.INTAKE
                  || elevator.getNearestHeight() == ElevatorHeight.BOTTOM;
          boolean atL4 =
              elevator.getNearestHeight() == ElevatorHeight.L4
                  || elevator.getNearestHeight() == ElevatorHeight.BARGE_ALAGAE;

          Pose2d robot = RobotContainer.poseSensorFusion.getEstimatedPosition();
          CoralPosition closestReef = CoralPosition.closestTo(robot);

          boolean nearReef =
              closestReef.getFirstStagePose().getTranslation().getDistance(robot.getTranslation())
                      < 0.2
                  && Math.abs(
                          closestReef
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

    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL2())
        .and(elevatorLock)
        .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L2));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL3())
        .and(elevatorLock)
        .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L3));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL4())
        .and(elevatorLock)
        .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.L4));
    // new Trigger(() -> DashboardUI.Overview.getControl().getBargeAlgae())
    //     .and(elevatorLock)
    //     .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.BARGE_ALAGAE));

    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL2())
        .and(() -> !elevatorLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL3())
        .and(() -> !elevatorLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL4())
        .and(() -> !elevatorLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));
    // new Trigger(() -> DashboardUI.Overview.getControl().getBargeAlgae())
    //     .and(() -> !elevatorLock.getAsBoolean())
    //     .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));

    new Trigger(() -> DashboardUI.Overview.getControl().getCoralShoot()).onTrue(new CoralShoot());

    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL1())
    //     .onTrue(HybridScoreCoral.deferred(ElevatorHeight.L1));
    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL2())
    //     .onTrue(HybridScoreCoral.deferred(ElevatorHeight.L2));
    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL3())
    //     .onTrue(HybridScoreCoral.deferred(ElevatorHeight.L3));
    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL4())
    //     .onTrue(HybridScoreCoral.deferred(ElevatorHeight.L4));

    new Trigger(() -> DashboardUI.Overview.getControl().getCoralGroundIntake())
        .toggleOnTrue(new CoralIntakeFromGroundToggled());

    new Trigger(() -> DashboardUI.Overview.getControl().getCoralSourceIntake())
        .onTrue(new CoralIntakeFromSource(true));

    new Trigger(() -> DashboardUI.Overview.getControl().getCoralSourceIntakeAuto())
        .debounce(0.2, DebounceType.kBoth)
        .whileTrue(
            new RepeatConditionallyCommand(
                new CoralIntakeFromSource(true)
                    .finallyDo(
                        () -> {
                          System.out.println("SOURCE ENDE!@!@H*&!*&@EQ#YQ#gyuaqd");
                          RobotContainer.elevatorHead.set(CoralShooterStates.OFF);
                          RobotContainer.coralIntake.toggleArm(IntakeArmState.UP);
                          RobotContainer.coralIntake.set(CoralIntakeState.OFF);
                        })
                    .asProxy(),
                () -> !RobotContainer.elevatorHead.hasCoral(),
                false));

    var coralScoreL1Cmd =
        Commands.either(
            Commands.either(
                new CoralIntakeShootL1().asProxy(),
                new CoralIntakeMoveL1().asProxy(),
                () -> coralIntake.getState() == CoralIntakeState.L1_DOWN),
            new CoralIntakeFromGroundUpL1()
                .asProxy()
                .beforeStarting(() -> CoralIntakeFromGroundToggled.isGoingToL1 = true),
            () -> coralIntake.getState() != CoralIntakeState.GROUND);
    // coralScoreL1Cmd.addRequirements(coralIntakeMoveToggleRequirement);

    new Trigger(() -> DashboardUI.Overview.getControl().getCoralIntakeScoreL1())
        .onTrue(coralScoreL1Cmd.asProxy());

    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralSourceIntake())
    //     .onTrue(HybridSource.deferred());

    BooleanSupplier algaeLock =
        () -> DashboardUI.Overview.getControl().getManualOverride() || !elevatorHead.hasCoral();

    new Trigger(() -> DashboardUI.Overview.getControl().getGroundAlgae())
        .and(algaeLock)
        .toggleOnTrue(new GroundAlgaeToggled(ElevatorHeight.GROUND_ALGAE));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorAlgaeLow())
        .and(algaeLock)
        .toggleOnTrue(new ElevatorAlgaeToggled(ElevatorHeight.LOW_REEF_ALGAE));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorAlgaeHigh())
        .and(algaeLock)
        .toggleOnTrue(new ElevatorAlgaeToggled(ElevatorHeight.HIGH_REEF_ALGAE));
    new Trigger(() -> DashboardUI.Overview.getControl().getScoreAlgae())
        .and(algaeLock)
        .onTrue(new ProcessorScore(true));

    new Trigger(() -> DashboardUI.Overview.getControl().getGroundAlgae())
        .and(() -> !algaeLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorAlgaeLow())
        .and(() -> !algaeLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorAlgaeHigh())
        .and(() -> !algaeLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kBothRumble, 1).withTimeout(0.1));
    new Trigger(() -> DashboardUI.Overview.getControl().getScoreAlgae())
        .and(() -> !algaeLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kLeftRumble, 1).withTimeout(0.1));

    new Trigger(() -> DashboardUI.Overview.getControl().getAutoAlign())
        .whileTrue(ReefAlign.alignClosest().repeatedly());

    new Trigger(() -> DashboardUI.Overview.getControl().getAutoScore())
        .and(
            () ->
                DashboardUI.Overview.getControl().getAutoScoreDirection()
                    == AutoScoreDirection.Left)
        .onTrue(new AutoScore(AutoScoreDirection.Left));

    new Trigger(() -> DashboardUI.Overview.getControl().getAutoScore())
        .and(
            () ->
                DashboardUI.Overview.getControl().getAutoScoreDirection()
                    == AutoScoreDirection.Right)
        .onTrue(new AutoScore(AutoScoreDirection.Right));

    new Trigger(() -> DashboardUI.Overview.getControl().getClimb())
        .onTrue(
            Commands.either(
                new ClimbUp(),
                new ClimbMove(ClimberState.Extend),
                () -> climber.getCurrentState() == ClimberState.Extend));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCommand == null) {
      autoCommand = new PlannedAuto();
    }
    return autoCommand;

    // return new InstantCommand()
    //     .andThen(climber.sysIdQuasistatic(Direction.kForward).andThen(new WaitCommand(0.4)))
    //     .andThen(climber.sysIdQuasistatic(Direction.kReverse).andThen(new WaitCommand(0.4)))
    //     .andThen(climber.sysIdDynamic(Direction.kForward).andThen(new WaitCommand(0.4)))
    //     .andThen(climber.sysIdDynamic(Direction.kReverse).andThen(new WaitCommand(0.4)));
  }

  public void testPeriodic() {
    DashboardUI.Test.testPeriodic();
  }

  public void simulationPeriodic() {
    updateSimulationBattery(drivetrain, elevator, elevatorHead, coralIntake);
    visionSim.update(model.getRobot());
  }

  public void updateSimulationBattery(PoweredSubsystem... subsystems) {
    double[] currents = new double[subsystems.length];
    for (int i = 0; i < subsystems.length; i++) {
      currents[i] = subsystems[i].getCurrentDrawAmps();
    }

    // RoboRioSim.setVInVoltage(
    //     MathUtil.clamp(BatterySim.calculateDefaultBatteryLoadedVoltage(currents), 0, 13));
  }

  public static boolean isInClimbMode() {
    return inClimbMode;
  }

  public static CoralLevel getCurrentCoralLevel() {
    if (Math.abs(coralIntake.getArmAngle() - Constants.CoralIntake.ARM_SCORE_L1)
        < Units.degreesToRadians(15)) {
      return CoralLevel.L1;
    }
    for (CoralLevel level : CoralLevel.values()) {
      if (elevator.getNearestHeight() == level.getHeight()) {
        return level;
      }
    }
    return null;
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
