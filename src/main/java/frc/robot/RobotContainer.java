// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ManualSwerve;
import frc.robot.control.IControlInput;
import frc.robot.control.SingleControl;
import frc.robot.subsystems.NavSensor;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

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
  // The robot's subsystems and commands are defined here...
  private Swerve _swerve;
  private NavSensor _nav;
  private List<Pair<Subsystem, Command>> _teleopPairs;

  private ManualSwerve _manualSwerve;

  private IControlInput _controlInput;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    _controlInput = new SingleControl(RobotMap.Control.SINGLE_GAMEPAD);

    // Init Swerve
    _swerve = new Swerve();

    // Init Nav
    _nav = new NavSensor();
    NavSensor.initNav();

    // Bindings and Teleop
    configureButtonBindings();
    initTeleopCommands();
  }

  private void initTeleopCommands() {
    _manualSwerve = new ManualSwerve(_swerve, _nav, _controlInput);
    _teleopPairs = new ArrayList<>();
    _teleopPairs.add(new Pair<Subsystem, Command>(_swerve, _manualSwerve));
  }

  public void teleopInit() {
    for (Pair<Subsystem, Command> c : _teleopPairs) {
      c.getFirst().setDefaultCommand(c.getSecond());
    }
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return _manualSwerve;
  }

  public void testSwerve() {
  }
}
