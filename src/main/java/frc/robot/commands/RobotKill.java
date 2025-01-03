// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class RobotKill extends Command {
  private Drivetrain _drivetrain;
  

  public RobotKill(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    _drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute () {
    SmartDashboard.putBoolean("robotkill", true);
    _drivetrain.stop();
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("robotkill", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}