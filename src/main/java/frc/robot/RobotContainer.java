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
  // Subsystems
  public static final PoseTracker poseTracker = new PoseTracker();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Channel channel = new Channel();
  public static final Acquisition acquisition = new Acquisition();
  public static final Shooter shooter = new Shooter();
  public static final Climbers climbers = new Climbers();
  public static final Crashbar crashbar = new Crashbar();
  public static final Photosensor photosensor = new Photosensor();
  public static final PCMCompressor pcmCompressor = new PCMCompressor();
  public static final Limelight limelight = new Limelight();

  // Autonomous
  private final AutoPath autoPath;
  private Command autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Sets up auto path
    autoPath = new AutoPath();

    // Sets up Control scheme chooser
    ShuffleboardUI.Overview.addControls(
        new JoystickXbox(2, 0), new DoubleXbox(0, 1), new DoubleXboxSpin(0, 1));

    // Bindings and Teleop
    configureButtonBindings();

    ShuffleboardPublisher.setup(
        poseTracker.nav,
        drivetrain,
        channel,
        acquisition,
        shooter,
        photosensor,
        pcmCompressor,
        limelight);
  }

  public void teleopInit() {
    // Sets default command for manual swerve. It is the only one right now
    drivetrain.setDefaultCommand(new ManualSwerve());
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
        .onTrue(new InstantCommand(pcmCompressor::disable))
        .onFalse(new InstantCommand(pcmCompressor::enable))
        .onTrue(new InstantCommand(() -> ShuffleboardUI.Overview.getControl().vibrate(1)))
        .onFalse(new InstantCommand(() -> ShuffleboardUI.Overview.getControl().vibrate(0)));

    // Notes triggers
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getAcquire())
        .toggleOnTrue(new AcquireSmart());
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getShootSpeaker())
        .toggleOnTrue(new ShootSpeaker());
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getShootAmp())
        .toggleOnTrue(new ShootAmp());
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getReverse())
        .whileTrue(new ManualReverse());

    // Manual triggers
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualShootAmp())
        .toggleOnTrue(new ManualShooter(Shooter.ShooterStates.AMP));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualShootSpeaker())
        .toggleOnTrue(new ManualShooter(Shooter.ShooterStates.SPEAKER));
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualCrashbar())
        .toggleOnTrue(new ManualCrashbar());
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualChannel())
        .whileTrue(new ManualChannel());
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getManualClimbers())
        .toggleOnTrue(new ManualClimbers());

    // Reset pose trigger
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getPoseReset())
        .onTrue(new InstantCommand(poseTracker::resetDriverPose));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCommand == null) {
      autoCommand = new PlannedAuto(autoPath);
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
    pcmCompressor.close();
    limelight.close();
  }
}
