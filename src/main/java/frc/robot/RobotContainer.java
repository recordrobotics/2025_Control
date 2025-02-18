package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
// WPILib imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.commands.CoralIntakeFromGround;
// Local imports
import frc.robot.commands.KillSpecified;
import frc.robot.commands.hybrid.HybridScoreCoral;
import frc.robot.commands.hybrid.HybridSource;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.commands.simulation.PlaceRandomGroundCoral;
import frc.robot.control.*;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.*;
import frc.robot.subsystems.io.sim.AlgaeGrabberSim;
import frc.robot.subsystems.io.sim.CoralIntakeSim;
import frc.robot.subsystems.io.sim.CoralShooterSim;
import frc.robot.subsystems.io.sim.ElevatorArmSim;
import frc.robot.subsystems.io.sim.ElevatorSim;
import frc.robot.subsystems.io.stub.AlgaeGrabberStub;
import frc.robot.subsystems.io.stub.CoralIntakeStub;
import frc.robot.subsystems.io.stub.CoralShooterStub;
import frc.robot.subsystems.io.stub.ElevatorArmStub;
import frc.robot.subsystems.io.stub.ElevatorStub;
import frc.robot.utils.AutoPath;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import java.util.Set;
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
  public static Lights lights;
  public static PowerDistributionPanel pdp;

  public static PhotonCamera camera;

  // Autonomous
  @SuppressWarnings("unused")
  private final AutoPath autoPath;

  @SuppressWarnings("unused")
  private Command autoCommand;

  private final GenericHID simulationController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.RobotState.getMode() == Mode.REAL) {
      drivetrain = new Drivetrain();
      poseTracker = new PoseTracker();
      limelight = new Limelight();
      elevator = new Elevator(new ElevatorStub(Constants.Elevator.kDt));
      elevatorArm = new ElevatorArm(new ElevatorArmStub(0.02));
      coralShooter = new CoralShooter(new CoralShooterStub(0.02));
      coralIntake = new CoralIntake(new CoralIntakeStub(0.02));
      algaeGrabber = new AlgaeGrabber(new AlgaeGrabberStub(0.02));
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
    new Trigger(() -> DashboardUI.Overview.getControl().getKill())
        .whileTrue(new KillSpecified(drivetrain))
        .onTrue(
            new InstantCommand(
                () ->
                    CommandScheduler.getInstance()
                        .cancel(
                            CommandScheduler.getInstance()
                                .requiring(
                                    drivetrain)))); // kill any commands that require drivetrain
    // (used for killing hybrid commands)

    // Reset pose trigger
    new Trigger(() -> DashboardUI.Overview.getControl().getPoseReset())
        .onTrue(new InstantCommand(poseTracker::resetDriverPose));

    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL1())
    //     .onTrue(new ElevatorMoveThenCoralShoot(ElevatorHeight.L1));
    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL2())
    //     .onTrue(new ElevatorMoveThenCoralShoot(ElevatorHeight.L2));
    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL3())
    //     .onTrue(new ElevatorMoveThenCoralShoot(ElevatorHeight.L3));
    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL4())
    //     .onTrue(new ElevatorMoveThenCoralShoot(ElevatorHeight.L4));

    new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL1())
        .onTrue(
            new DeferredCommand(
                () -> new HybridScoreCoral(ElevatorHeight.L1),
                Set.of(drivetrain, elevator, coralShooter)));
    new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL2())
        .onTrue(
            new DeferredCommand(
                () -> new HybridScoreCoral(ElevatorHeight.L2),
                Set.of(drivetrain, elevator, coralShooter)));
    new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL3())
        .onTrue(
            new DeferredCommand(
                () -> new HybridScoreCoral(ElevatorHeight.L3),
                Set.of(drivetrain, elevator, coralShooter)));
    new Trigger(() -> DashboardUI.Overview.getControl().getCoralShootL4())
        .onTrue(
            new DeferredCommand(
                () -> new HybridScoreCoral(ElevatorHeight.L4),
                Set.of(drivetrain, elevator, coralShooter)));

    new Trigger(() -> DashboardUI.Overview.getControl().getCoralGroundIntake())
        .onTrue(new CoralIntakeFromGround());

    // new Trigger(() -> DashboardUI.Overview.getControl().getCoralSourceIntake())
    //     .onTrue(new CoralIntakeFromSource());

    new Trigger(() -> DashboardUI.Overview.getControl().getCoralSourceIntake())
        .onTrue(
            new DeferredCommand(
                () -> new HybridSource(), Set.of(drivetrain, elevator, coralShooter, coralIntake)));

    new Trigger(() -> DashboardUI.Overview.getControl().getReefAlgae());
    new Trigger(() -> DashboardUI.Overview.getControl().getScoreAlgae());
    new Trigger(() -> DashboardUI.Overview.getControl().getClimb());

    // Simulation control commands
    if (Constants.RobotState.getMode() == Mode.SIM) {
      new Trigger(() -> simulationController.getRawButton(1)).onTrue(new PlaceRandomGroundCoral());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // if (autoCommand == null) {
    //   autoCommand = new PlannedAuto();
    // }
    // return autoCommand;

    return new InstantCommand()
        .andThen(coralIntake.sysIdQuasistaticArm(Direction.kForward).andThen(new WaitCommand(0.2)))
        .andThen(coralIntake.sysIdQuasistaticArm(Direction.kForward).andThen(new WaitCommand(0.2)))
        .andThen(coralIntake.sysIdQuasistaticArm(Direction.kReverse).andThen(new WaitCommand(0.2)))
        .andThen(coralIntake.sysIdDynamicArm(Direction.kForward).andThen(new WaitCommand(0.2)))
        .andThen(coralIntake.sysIdDynamicArm(Direction.kReverse).andThen(new WaitCommand(0.2)));
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

    // RoboRioSim.setVInVoltage(
    //     MathUtil.clamp(BatterySim.calculateDefaultBatteryLoadedVoltage(currents), 0, 13));
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
