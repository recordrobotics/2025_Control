// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Crashbar;

import java.util.EnumSet;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.KillSpecified;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.commands.manual.ManualAcquisition;
import frc.robot.commands.hybrid.FindNote;
import frc.robot.commands.hybrid.NoteOrient;
import frc.robot.commands.manual.ManualClimbers;
import frc.robot.commands.manual.ManualCrashbar;
import frc.robot.commands.manual.ManualShooter;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.commands.manual.ManualReverse;
import frc.robot.commands.notes.AcquireSmart;
import frc.robot.commands.notes.ShootAmp;
import frc.robot.commands.notes.ShootSpeaker;
import frc.robot.utils.ShuffleboardChoosers;
import frc.robot.control.DoubleXbox;
import frc.robot.control.DoubleXboxSpin;
import frc.robot.control.JoystickXbox;
import frc.robot.subsystems.AutoPath;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PCMCompressor;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Acquisition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public enum AutoName {
    Speaker_2_note("2 Note Speaker"),
    Speaker_4_note("4 Note Speaker"),
    Amp_Speaker_2_note("Amp and Speaker (2)"),
    Amp_Speaker_1_note("Amp and Speaker"),
    Amp("Amp"),
    FarSpeaker("Far Speaker"),
    DiagLeftOneNote("DiagLeftOneNote"),
    DiagJustShoot("DiagJustShoot"),
    Speaker_3_Note("3NoteSpeaker");

    private String pathref;

    public String getPathRef() {
      return pathref;
    }

    private AutoName(String pathplannerRef) {
      pathref = pathplannerRef;
    }
  }

  // The robot's subsystems and commands are defined here
  private final Drivetrain _drivetrain;
  private final Shooter _shooter;
  private final Crashbar _crashbar;
  private final Climbers _climbers;
  private final Acquisition _acquisition;
  private final Channel _channel;
  private final Photosensor _photosensor;
  private final PCMCompressor _compressor;
  private final Vision _vision;

  // Autonomous
  private final AutoPath _autoPath;
  private Command autoCommand;

  // Manual (default) commands
  private ManualSwerve _manualSwerve;

  // Control
  private JoystickXbox _joystickXbox;
  private DoubleXbox _doubleXbox;
  private DoubleXboxSpin _doubleXboxSpin;

  public static SendableChooser<AutoName> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Init subsystems
    _drivetrain = new Drivetrain();
    _channel = new Channel();
    _acquisition = new Acquisition();
    _shooter = new Shooter();
    _crashbar = new Crashbar();
    _photosensor = new Photosensor();
    _climbers = new Climbers();
    _compressor = new PCMCompressor();

    _manualSwerve = new ManualSwerve(_drivetrain);
    _vision = new Vision();

    // Sets up auto chooser
    _autoPath = new AutoPath(_drivetrain, _acquisition, _photosensor, _channel, _shooter, _crashbar);
    _autoPath.putAutoChooser();

    // Creates control input & manual swerve object, adds it to _teleopPairs
    _joystickXbox = new JoystickXbox(2, 0);
    _doubleXbox = new DoubleXbox(0, 1);
    _doubleXboxSpin = new DoubleXboxSpin(0, 1);
    // Sets up Control scheme chooser
    ShuffleboardChoosers.initialize(_joystickXbox, _doubleXbox, _doubleXboxSpin);

    EnumSet.allOf(AutoName.class)
        .forEach(v -> autoChooser.addOption(v.pathref, v));
    autoChooser.setDefaultOption(AutoName.Speaker_2_note.pathref, AutoName.Speaker_2_note);

    ShuffleboardTab tab = ShuffleboardUI.Autonomous.getTab();
    var autoWidget = tab.add("Auto Code", autoChooser);
    autoWidget.withWidget(BuiltInWidgets.kComboBoxChooser);
    autoWidget.withSize(3, 1);
    autoWidget.withPosition(6, 1);

    // Bindings and Teleop
    configureButtonBindings();
  }

  public void teleopInit() {
    // Sets default command for manual swerve. It is the only one right now
    _drivetrain.setDefaultCommand(_manualSwerve);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Command to kill robot
    new Trigger(()->ShuffleboardChoosers.getDriveControl().getKillAuto()).
      whileTrue(new KillSpecified(_drivetrain, _acquisition, _channel, _shooter, _crashbar, _climbers));


    // Smart triggers
    new Trigger(()->ShuffleboardChoosers.getDriveControl().getAcquire()).
      toggleOnTrue(new AcquireSmart(_acquisition, _channel, _photosensor, _shooter));

    new Trigger(()->ShuffleboardChoosers.getDriveControl().getShootSpeaker()).
      toggleOnTrue(new ShootSpeaker(_channel, _shooter));

    new Trigger(()->ShuffleboardChoosers.getDriveControl().getShootAmp()).
      toggleOnTrue(new ShootAmp(_channel, _shooter, _crashbar));

    new Trigger(()->ShuffleboardChoosers.getDriveControl().getReverse()).
      whileTrue(new ManualReverse(_acquisition, _channel));

    new Trigger(() ->  ShuffleboardChoosers.getDriveControl().turnToNote()).onTrue(new NoteOrient(_drivetrain, _vision, _photosensor, () ->  ShuffleboardChoosers.getDriveControl().turnToNote()));


    // Manual triggers
    new Trigger(()->ShuffleboardChoosers.getDriveControl().getManualShootAmp()).
      toggleOnTrue(new ManualShooter(_shooter, ShooterStates.AMP));

    new Trigger(()->ShuffleboardChoosers.getDriveControl().getManualShootSpeaker()).
			toggleOnTrue(new ManualShooter(_shooter, ShooterStates.SPEAKER));

    new Trigger(()->ShuffleboardChoosers.getDriveControl().getManualCrashbar()).
			toggleOnTrue(new ManualCrashbar(_crashbar));

    new Trigger(()->ShuffleboardChoosers.getDriveControl().getManualAcquisition()).
			whileTrue(new ManualAcquisition(_acquisition, _channel));

    new Trigger(()->ShuffleboardChoosers.getDriveControl().getManualClimbers()).
			toggleOnTrue(new ManualClimbers(_climbers));


    // Reset pose trigger
    new Trigger(()->ShuffleboardChoosers.getDriveControl().getPoseReset()).
			onTrue(new InstantCommand(_drivetrain::resetPose));
    
  }

  public void testPeriodic() {
    _shooter.testPeriodic();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoCommand == null) {
      autoCommand = new PlannedAuto(_drivetrain, _autoPath);
    }
    return autoCommand;
  }
}