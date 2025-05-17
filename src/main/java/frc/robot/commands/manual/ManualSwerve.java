// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.assists.DrivetrainControl;
import org.littletonrobotics.junction.Logger;

/** An example command that uses an example subsystem. */
public class ManualSwerve extends Command {

  /**
   * @param drivetrain
   */
  public ManualSwerve() {
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AbstractControl controls = DashboardUI.Overview.getControl();

    DrivetrainControl drivetrainControl = controls.getDrivetrainControl();

    int applyCount = 0;
    for (var assist : RobotContainer.assits) {
      if (assist.isEnabled()) {
        if (assist.apply(drivetrainControl)) {
          applyCount++;
        }
      }
    }

    Logger.recordOutput("ManualSwerve/AssistsApplied", applyCount);

    RobotContainer.drivetrain.drive(drivetrainControl.toChassisSpeeds());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
