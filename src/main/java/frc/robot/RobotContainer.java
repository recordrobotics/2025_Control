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
    compressor = new PCMCompressor();
    limelight = new Limelight();

    // this is very cursed but it is less cursed than other ways to do it, so don't touch
    PoseTracker.instance = new PoseTracker(drivetrain, limelight);

    // Sets up auto path
    autoPath = new AutoPath(drivetrain);

    // Sets up Control scheme chooser
    ShuffleboardUI.Overview.addControls(
        new JoystickXbox(2, 0), new DoubleXbox(0, 1), new DoubleXboxSpin(0, 1));

    // Bindings and Teleop
    configureButtonBindings();

    ShuffleboardPublisher.setup(nav, drivetrain, compressor, limelight);
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
        .whileTrue(new KillSpecified(drivetrain));

    // Command to kill compressor
    new Trigger(() -> ShuffleboardUI.Overview.getControl().getKillCompressor())
        .onTrue(new InstantCommand(compressor::disable))
        .onFalse(new InstantCommand(compressor::enable))
        .onTrue(new InstantCommand(() -> ShuffleboardUI.Overview.getControl().vibrate(1)))
        .onFalse(new InstantCommand(() -> ShuffleboardUI.Overview.getControl().vibrate(0)));

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
    compressor.close();
    limelight.close();
  }
}
