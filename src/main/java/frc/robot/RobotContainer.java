package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
// WPILib imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// Local imports
import frc.robot.commands.KillSpecified;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.control.*;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.*;
import frc.robot.subsystems.io.CoralIntakeSim;
import frc.robot.utils.AutoPath;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.ShuffleboardPublisher;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final RobotModel model = new RobotModel();

  // Subsystems
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final PoseTracker poseTracker = new PoseTracker();
  public static final Limelight limelight = new Limelight();
  public static final Elevator elevator = null; // new Elevator();
  public static final CoralShooter coralShooter = null; // = new CoralShooter();
  public static final CoralIntake coralIntake = new CoralIntake(new CoralIntakeSim(0.02));
  public static final ElevatorAlgae elevatorAlgae = null; // new ElevatorAlgae();
  public static final GroundAlgae groundAlgae = null; // new GroundAlgae();
  public static final Lights lights = new Lights();
  public static final PowerDistributionPanel pdp = new PowerDistributionPanel();

  public static final PhotonCamera camera = new PhotonCamera("photonvision");

  // Autonomous
  @SuppressWarnings("unused")
  private final AutoPath autoPath;

  @SuppressWarnings("unused")
  private Command autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Sets up auto path
    autoPath = new AutoPath();

    DashboardUI.Autonomous.setupAutoChooser();

    // Sets up Control scheme chooser
    DashboardUI.Overview.addControls(new JoystickXbox(2, 0));

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

    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralShootL1())
    //     .onTrue(new ElevatorMove(ElevatorHeight.L1));
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralShootL2())
    //     .onTrue(new ElevatorMove(ElevatorHeight.L2));
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralShootL3())
    //     .onTrue(new ElevatorMove(ElevatorHeight.L3));
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralShootL4())
    //     .onTrue(new ElevatorMove(ElevatorHeight.L4));
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralGroundIntake())
    //     .onTrue(new CoralIntakeFromGround());
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralSourceIntake())
    //     .onTrue(new CoralIntakeFromSource());
    // // TODO: new Trigger(() -> ShuffleboardUI.Overview.getControl().getIntakeAlgae());
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getIntakeAlgae())
    //     .onTrue(new LightSignal(LightMode.CHASE));
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getReefAlgae())
    //     .onTrue(new LightSignal(LightMode.RAINBOW));
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralShootL1())
    //     .onTrue(new ElevatorMove(ElevatorHeight.L1));
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralShootL2())
    //     .onTrue(new ElevatorMove(ElevatorHeight.L2));
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralShootL3())
    //     .onTrue(new ElevatorMove(ElevatorHeight.L3));
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralShootL4())
    //     .onTrue(new ElevatorMove(ElevatorHeight.L4));
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralGroundAcquire())
    //     .onTrue(new CoralIntakeFromGround());
    // new Trigger(() -> ShuffleboardUI.Overview.getControl().getCoralSourceAcquire())
    //     .onTrue(new CoralIntakeFromSource());
    // // TODO: new Trigger(() -> ShuffleboardUI.Overview.getControl().getAcquireAlgae());
    // new Trigger(() -> DashboardUI.Overview.getControl().getIntakeAlgae());
    // new Trigger(() -> DashboardUI.Overview.getControl().getReefAlgae());
    // TODO: new Trigger(() -> ShuffleboardUI.Overview.getControl().getReefAlgae());
    // TODO: new Trigger(() -> ShuffleboardUI.Overview.getControl().getScoreAlgae());
    // TODO: new Trigger(() -> ShuffleboardUI.Overview.getControl().getClimb());
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

    return drivetrain
        .run(() -> drivetrain.drive(new DriveCommandData()))
        .withTimeout(0.4)
        .andThen(drivetrain.sysIdQuasistaticDriveMotors(Direction.kForward))
        .andThen(drivetrain.run(() -> drivetrain.drive(new DriveCommandData())).withTimeout(0.4))
        .andThen(drivetrain.sysIdQuasistaticDriveMotors(Direction.kReverse))
        .andThen(drivetrain.run(() -> drivetrain.drive(new DriveCommandData())).withTimeout(0.4))
        .andThen(drivetrain.sysIdDynamicDriveMotors(Direction.kForward))
        .andThen(drivetrain.run(() -> drivetrain.drive(new DriveCommandData())).withTimeout(0.4))
        .andThen(drivetrain.sysIdDynamicDriveMotors(Direction.kReverse));
  }

  public void testPeriodic() {
    DashboardUI.Test.testPeriodic();
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
    drivetrain.close();
    limelight.close();
    // elevator.close();
    // coralShooter.close();
    coralIntake.close();
    // elevatorAlgae.close();
    // groundAlgae.close();
    pdp.close();
  }
}
