package frc.robot;

// WPILib imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// Local imports
import frc.robot.commands.KillSpecified;
import frc.robot.commands.auto.*;
import frc.robot.commands.manual.*;
import frc.robot.commands.notes.*;
import frc.robot.control.*;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.subsystems.*;
import frc.robot.utils.AutoPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here
  private final NavSensor nav;
  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private final Crashbar crashbar;
  private final Climbers climbers;
  private final Acquisition acquisition;
  private final Channel channel;
  private final Photosensor photosensor;
  private final PCMCompressor compressor;
  private final Limelight limelight;

  // Autonomous
  private final AutoPath autoPath;
  private Command autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Init subsystems
    nav = new NavSensor();
    drivetrain = new Drivetrain();
    channel = new Channel();
    acquisition = new Acquisition();
    shooter = new Shooter();
    crashbar = new Crashbar();
    photosensor = new Photosensor();
    climbers = new Climbers();
    compressor = new PCMCompressor();
    limelight = new Limelight();

    // this is very cursed but it is less cursed than other ways to do it, so don't touch
    PoseTracker.instance = new PoseTracker(drivetrain, limelight);

    // Sets up auto path
    autoPath = new AutoPath(drivetrain, acquisition, photosensor, channel, shooter, crashbar);

    // Sets up Control scheme chooser
    ShuffleboardUI.Overview.addControls(
        new JoystickXbox(2, 0), new DoubleXbox(0, 1), new DoubleXboxSpin(0, 1));

    // Bindings and Teleop
    configureButtonBindings();

    ShuffleboardPublisher.setup(
        nav, drivetrain, channel, acquisition, shooter, photosensor, compressor, limelight);
  }

  public void teleopInit() {
    // Sets default command for manual swerve. It is the only one right now
    drivetrain.setDefaultCommand(new ManualSwerve(drivetrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Command to kill robot
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getKillAuto())
        .whileTrue(
            new KillSpecified(drivetrain, acquisition, channel, shooter, crashbar, climbers));

    // Command to kill compressor
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getKillCompressor())
        .onTrue(new InstantCommand(compressor::disable))
        .onFalse(new InstantCommand(compressor::enable))
        .onTrue(new InstantCommand(() -> ShuffleboardUI.Overview.getControl().vibrate(1)))
        .onFalse(new InstantCommand(() -> ShuffleboardUI.Overview.getControl().vibrate(0)));

    // Notes triggers
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getAcquire())
        .toggleOnTrue(new AcquireSmart(acquisition, channel, photosensor, shooter));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getShootSpeaker())
        .toggleOnTrue(new ShootSpeaker(channel, shooter));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getShootAmp())
        .toggleOnTrue(new ShootAmp(channel, shooter, crashbar));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getReverse())
        .whileTrue(new ManualReverse(acquisition, channel));

    // Manual triggers
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualShootAmp())
        .toggleOnTrue(new ManualShooter(shooter, Shooter.ShooterStates.AMP));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualShootSpeaker())
        .toggleOnTrue(new ManualShooter(shooter, Shooter.ShooterStates.SPEAKER));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualCrashbar())
        .toggleOnTrue(new ManualCrashbar(crashbar));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualChannel())
        .whileTrue(new ManualChannel(channel));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualClimbers())
        .toggleOnTrue(new ManualClimbers(climbers));

    // Reset pose trigger
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getPoseReset())
        .onTrue(new InstantCommand(PoseTracker::resetDriverPose));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCommand == null) {
      autoCommand = new PlannedAuto(drivetrain, autoPath);
    }
    return autoCommand;
  }

  public void testPeriodic() {
    ShuffleboardUI.Test.testPeriodic();
  }

  /** frees up all hardware allocations */
  public void close() {
    drivetrain.close();
    channel.close();
    acquisition.close();
    shooter.close();
    crashbar.close();
    photosensor.close();
    climbers.close();
    compressor.close();
    limelight.close();
  }
}
