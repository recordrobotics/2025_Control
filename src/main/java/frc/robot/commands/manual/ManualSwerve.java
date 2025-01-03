// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;
import frc.robot.Constants;
import frc.robot.control.DoubleControl;
import frc.robot.control.JoystickOrientation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.drivemodes.AutoOrient;
import frc.robot.utils.drivemodes.DefaultSpin;
import frc.robot.utils.drivemodes.XboxSpin;


import frc.robot.utils.drivemodes.DefaultDrive;
import frc.robot.utils.drivemodes.TabletDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ManualSwerve extends Command {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  // Creates Drivetrain and Controls variables
  private Drivetrain _drivetrain;
  private DoubleControl _controls;

  // Sets up sendable chooser for drivemode
  public enum DriveMode {
    Robot, Field, Tablet
  }

  private SendableChooser<DriveMode> driveMode = new SendableChooser<DriveMode>();
  private SendableChooser<JoystickOrientation> joystickOrientation = new SendableChooser<JoystickOrientation>();

  // Sets up spin modes
  public AutoOrient autoOrient = new AutoOrient();
  public XboxSpin xboxSpin = new XboxSpin();

  /**
   * @param drivetrain
   */
  public ManualSwerve(Drivetrain drivetrain, DoubleControl controls) {

    // Init variables
    _drivetrain = drivetrain;
    _controls = controls;
    addRequirements(drivetrain);

    // Creates selector on SmartDashboard for drivemode
    // driveMode.addOption("AutoOrient", DriveMode.AutoOrient);
    driveMode.addOption("Robot", DriveMode.Robot);
    driveMode.addOption("Field", DriveMode.Field);
    driveMode.addOption("Tablet", DriveMode.Tablet);
    driveMode.setDefaultOption("Field", DriveMode.Field);

    joystickOrientation.addOption("X Axis", JoystickOrientation.XAxisTowardsTrigger);
    joystickOrientation.addOption("Y Axis", JoystickOrientation.YAxisTowardsTrigger);
    joystickOrientation.setDefaultOption("X Axis", JoystickOrientation.XAxisTowardsTrigger);

    // puts selector data on Smartdashboard
    SmartDashboard.putData("Drive Mode", driveMode);
    SmartDashboard.putData("Joystick Orientation", joystickOrientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    _controls.setJoystickOrientation(joystickOrientation.getSelected());
    
    // Gets swerve position and sets to field position
    Pose2d swerve_position = _drivetrain.poseFilter.getEstimatedPosition();

    // Puts robot position information on shuffleboard
    SmartDashboard.putNumber("Rotation", swerve_position.getRotation().getDegrees());
    SmartDashboard.putNumber("X", swerve_position.getX());
    SmartDashboard.putNumber("Y", swerve_position.getY());

    // Control to reset pose if reset button is pressed
    if (_controls.getResetPressed()) {
      _drivetrain.resetPose(DriverStationUtils.getCurrentAlliance() == Alliance.Red
                              ? Constants.FieldConstants.TEAM_RED_STARTING_POSE
                              : Constants.FieldConstants.TEAM_BLUE_STARTING_POSE
                            );
    }

    // Sets up spin
    double spin;

    // Auto-orient function
    if (autoOrient.shouldExecute(_controls)) {
      spin = autoOrient.calculate(_controls, swerve_position);
    }
    else if (xboxSpin.shouldExecute(_controls)) {
      spin = xboxSpin.calculate(_controls, swerve_position);
    }
    else {
      spin = DefaultSpin.calculate(_controls);
    }

    // Sets up driveCommandData object
    DriveCommandData driveCommandData;

    switch (driveMode.getSelected()) {
      case Tablet:
        driveCommandData = TabletDrive.calculate(_controls, spin, swerve_position);
        break;
      case Robot:
        driveCommandData = DefaultDrive.calculate(_controls, spin, swerve_position, false);
        break;
      default:
        driveCommandData = DefaultDrive.calculate(_controls, spin, swerve_position, true);
        break;
    }

    // Drive command
    _drivetrain.drive(driveCommandData);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}