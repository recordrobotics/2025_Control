package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
// WPILib imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.RobotAlignPose;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.commands.Align;
import frc.robot.commands.CoralIntakeFromGroundToggled;
import frc.robot.commands.CoralIntakeFromGroundUp;
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
import frc.robot.commands.VibrateXbox;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.commands.hybrid.AutoScore;
import frc.robot.commands.manual.ManualElevator;
import frc.robot.commands.manual.ManualElevatorArm;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.commands.simulation.PlaceRandomGroundAlgae;
import frc.robot.commands.simulation.PlaceRandomGroundCoral;
import frc.robot.control.*;
import frc.robot.control.AbstractControl.AutoScoreDirection;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;
import frc.robot.subsystems.io.real.AlgaeGrabberReal;
import frc.robot.subsystems.io.real.CoralIntakeReal;
import frc.robot.subsystems.io.real.CoralShooterReal;
import frc.robot.subsystems.io.real.ElevatorArmReal;
import frc.robot.subsystems.io.real.ElevatorReal;
import frc.robot.subsystems.io.sim.AlgaeGrabberSim;
import frc.robot.subsystems.io.sim.ClimberSim;
import frc.robot.subsystems.io.sim.CoralIntakeSim;
import frc.robot.subsystems.io.sim.CoralShooterSim;
import frc.robot.subsystems.io.sim.ElevatorArmSim;
import frc.robot.subsystems.io.sim.ElevatorSim;
import frc.robot.subsystems.io.stub.ClimberStub;
import frc.robot.utils.AutoPath;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import java.util.function.BooleanSupplier;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final RobotModel model = new RobotModel();

  public static Drivetrain drivetrain;
  public static PoseTracker poseTracker;
  public static Limelight limelight;
  public static Elevator elevator;
  public static ElevatorArm elevatorArm;
  public static CoralShooter coralShooter;
  public static CoralIntake coralIntake;
  public static AlgaeGrabber algaeGrabber;
  public static Climber climber;
  public static Lights lights;
  public static PowerDistributionPanel pdp;

  public static PhotonCamera camera;

  // Autonomous
  @SuppressWarnings("unused")
  private final AutoPath autoPath;

  private Command autoCommand;

  private final GenericHID simulationController;

  public static class ElevatorMoveToggleRequirement extends SubsystemBase {}

  public static ElevatorMoveToggleRequirement elevatorMoveToggleRequirement =
      new ElevatorMoveToggleRequirement();

  public static class CoralIntakeMoveToggleRequirement extends SubsystemBase {}

  public static CoralIntakeMoveToggleRequirement coralIntakeMoveToggleRequirement =
      new CoralIntakeMoveToggleRequirement();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.RobotState.getMode() == Mode.REAL) {
      drivetrain = new Drivetrain();
      poseTracker = new PoseTracker();
      limelight = new Limelight();
      elevator = new Elevator(new ElevatorReal(Constants.Elevator.kDt));
      elevatorArm = new ElevatorArm(new ElevatorArmReal(0.02));
      coralShooter = new CoralShooter(new CoralShooterReal(0.02));
      coralIntake = new CoralIntake(new CoralIntakeReal(0.02));
      algaeGrabber = new AlgaeGrabber(new AlgaeGrabberReal(0.02));
      climber = new Climber(new ClimberStub(0.02));
      lights = new Lights();
      pdp = new PowerDistributionPanel();
      camera = new PhotonCamera("photonvision");
    } else {
      drivetrain = new Drivetrain();
      poseTracker = new PoseTracker();
      limelight = new Limelight();
      elevator = new Elevator(new ElevatorSim(Constants.Elevator.kDt));
      elevatorArm = new ElevatorArm(new ElevatorArmSim(0.02));
      coralShooter = new CoralShooter(new CoralShooterSim(0.02));
      coralIntake = new CoralIntake(new CoralIntakeSim(0.02));
      algaeGrabber = new AlgaeGrabber(new AlgaeGrabberSim(0.02));
      climber = new Climber(new ClimberSim(0.02));
      lights = new Lights();
      pdp = new PowerDistributionPanel();
      camera = new PhotonCamera("photonvision");
    }

    // Sets up auto path
    autoPath = new AutoPath();

    DashboardUI.Autonomous.setupAutoChooser();

    // Sets up Control scheme chooser
    DashboardUI.Overview.addControls(new JoystickXbox(2, 0), new JoystickXboxKeypad(2, 0, 3));

    if (Constants.RobotState.getMode() == Mode.SIM) {
      simulationController = new GenericHID(4);
    } else {
      simulationController = null;
    }

    // Bindings and Teleop
    configureButtonBindings();

    ShuffleboardPublisher.setup(poseTracker.nav, drivetrain, limelight);

    drivetrain.setDefaultCommand(new ManualSwerve());
    elevator.setDefaultCommand(new ManualElevator());
    elevatorArm.setDefaultCommand(new ManualElevatorArm());

    // camera exposure fix
    camera.setDriverMode(false);
    NetworkTableInstance.getDefault().flush();
    camera.setDriverMode(true);
    NetworkTableInstance.getDefault().flush();
  }

  public void teleopInit() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Command to kill robot
    // KillSpecified requires the subsystems, which cancels the commands that require them (which is
    // all the commands)
    new Trigger(() -> DashboardUI.Overview.getControl().getKill())
        .whileTrue(
            new KillSpecified(
                drivetrain,
                elevator,
                elevatorArm,
                coralShooter,
                coralIntake,
                algaeGrabber,
                climber));

    // Reset pose trigger
    new Trigger(() -> DashboardUI.Overview.getControl().getPoseReset())
        .onTrue(new InstantCommand(poseTracker::resetDriverPose));
    new Trigger(() -> DashboardUI.Overview.getControl().getLimelightReset())
        .onTrue(new InstantCommand(poseTracker::resetFullLimelight));
    new Trigger(() -> DashboardUI.Autonomous.getResetLocation())
        .onTrue(new InstantCommand(poseTracker::resetStartingPose).ignoringDisable(true));
    new Trigger(() -> DashboardUI.Autonomous.getLimelightRotation())
        .onTrue(new InstantCommand(poseTracker::resetToLimelight).ignoringDisable(true));

    BooleanSupplier elevatorLock =
        () -> {
          boolean atBottom =
              elevator.getNearestHeight() == ElevatorHeight.INTAKE
                  || elevator.getNearestHeight() == ElevatorHeight.BOTTOM;
          boolean atL4 =
              elevator.getNearestHeight() == ElevatorHeight.L4
                  || elevator.getNearestHeight() == ElevatorHeight.BARGE_ALAGAE;

          Pose2d robot = RobotContainer.poseTracker.getEstimatedPosition();
          RobotAlignPose closestReef = RobotAlignPose.closestReefTo(robot, 0.2);

          boolean nearReef =
              closestReef != null
                  && Math.abs(
                          closestReef
                              .getPose()
                              .getRotation()
                              .minus(robot.getRotation())
                              .getDegrees())
                      < 80;

          return DashboardUI.Overview.getControl().getManualOverride()
              || (atBottom && coralShooter.coralReady())
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
    new Trigger(() -> DashboardUI.Overview.getControl().getBargeAlgae())
        .and(elevatorLock)
        .toggleOnTrue(new ElevatorReefToggled(ElevatorHeight.BARGE_ALAGAE));

    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL2())
        .and(() -> !elevatorLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL3())
        .and(() -> !elevatorLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorL4())
        .and(() -> !elevatorLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));
    new Trigger(() -> DashboardUI.Overview.getControl().getBargeAlgae())
        .and(() -> !elevatorLock.getAsBoolean())
        .onTrue(new VibrateXbox(RumbleType.kRightRumble, 1).withTimeout(0.1));

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

    var coralScoreL1Cmd =
        Commands.either(
            Commands.either(
                new CoralIntakeShootL1(),
                new CoralIntakeMoveL1(),
                () -> coralIntake.getArmState() == IntakeArmStates.SCORE_L1),
            new CoralIntakeFromGroundUp(false),
            () -> coralIntake.getArmAngle() >= Constants.CoralIntake.ARM_SCORE_L1 - 0.1);
    coralScoreL1Cmd.addRequirements(coralIntakeMoveToggleRequirement);

    new Trigger(() -> DashboardUI.Overview.getControl().getCoralIntakeScoreL1())
        .onTrue(coralScoreL1Cmd);

    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralSourceIntake())
    //     .onTrue(HybridSource.deferred());

    new Trigger(() -> DashboardUI.Overview.getControl().getGroundAlgae())
        .toggleOnTrue(new GroundAlgaeToggled(ElevatorHeight.GROUND_ALGAE));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorAlgaeLow())
        .toggleOnTrue(new ElevatorAlgaeToggled(ElevatorHeight.LOW_REEF_ALGAE));
    new Trigger(() -> DashboardUI.Overview.getControl().getElevatorAlgaeHigh())
        .toggleOnTrue(new ElevatorAlgaeToggled(ElevatorHeight.HIGH_REEF_ALGAE));
    new Trigger(() -> DashboardUI.Overview.getControl().getScoreAlgae())
        .onTrue(new ProcessorScore());

    new Trigger(() -> DashboardUI.Overview.getControl().getAutoAlign())
        .whileTrue(Align.create(0.01, 0.05, false, 1));

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

    // Simulation control commands
    if (Constants.RobotState.getMode() == Mode.SIM) {
      new Trigger(() -> simulationController.getRawButton(1)).onTrue(new PlaceRandomGroundCoral());
      new Trigger(() -> simulationController.getRawButton(2)).onTrue(new PlaceRandomGroundAlgae());
    }
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
    //     .andThen(
    //         drivetrain
    //             .sysIdQuasistaticDriveMotors(Direction.kForward)
    //             .andThen(new WaitCommand(0.4)))
    //     .andThen(
    //         drivetrain
    //             .sysIdQuasistaticDriveMotors(Direction.kForward)
    //             .andThen(new WaitCommand(0.4)))
    //     .andThen(
    //         drivetrain
    //             .sysIdQuasistaticDriveMotors(Direction.kReverse)
    //             .andThen(new WaitCommand(0.4)))
    //     .andThen(
    //         drivetrain.sysIdDynamicDriveMotors(Direction.kForward).andThen(new WaitCommand(0.4)))
    //     .andThen(
    //         drivetrain.sysIdDynamicDriveMotors(Direction.kReverse).andThen(new
    // WaitCommand(0.4)));
  }

  public void testPeriodic() {
    DashboardUI.Test.testPeriodic();
  }

  public void simulationPeriodic() {
    updateSimulationBattery(drivetrain, elevator, coralShooter, coralIntake, algaeGrabber);
  }

  public void updateSimulationBattery(PoweredSubsystem... subsystems) {
    double[] currents = new double[subsystems.length];
    for (int i = 0; i < subsystems.length; i++) {
      currents[i] = subsystems[i].getCurrentDrawAmps();
    }

    RoboRioSim.setVInVoltage(
        MathUtil.clamp(BatterySim.calculateDefaultBatteryLoadedVoltage(currents), 0, 13));
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    drivetrain.close();
    limelight.close();
    elevator.close();
    elevatorArm.close();
    coralShooter.close();
    coralIntake.close();
    algaeGrabber.close();
    pdp.close();
  }
}
