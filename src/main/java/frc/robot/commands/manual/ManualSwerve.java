// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.DriveCommandData;
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
    double elevatorHeightFromTop =
        Constants.Elevator.MAX_HEIGHT - RobotContainer.elevator.getCurrentHeight();
    double maxAcceleration =
        Constants.Elevator.DRIVETRAIN_ACCELERATION_PER_ELEVATOR_METER_FROM_TOP
                * elevatorHeightFromTop
            + Constants.Elevator.DRIVETRAIN_ACCELERATION_WHEN_AT_TOP;

    AbstractControl controls = DashboardUI.Overview.getControl();

    DriveCommandData driveCommandData = controls.getDriveCommandData();

    ChassisSpeeds currentSpeeds = RobotContainer.drivetrain.getChassisSpeeds();
    ChassisSpeeds commandedSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveCommandData.xSpeed,
            driveCommandData.ySpeed,
            driveCommandData.rot,
            RobotContainer.poseTracker.getEstimatedPosition().getRotation());
    ChassisSpeeds diffSpeeds = commandedSpeeds.minus(currentSpeeds);
    double translationalAcceleration =
        Math.sqrt(
            diffSpeeds.vxMetersPerSecond * diffSpeeds.vxMetersPerSecond
                + diffSpeeds.vyMetersPerSecond * diffSpeeds.vyMetersPerSecond); // m/s^2
    if (translationalAcceleration > maxAcceleration) {
      double percentOver = translationalAcceleration / maxAcceleration;
      driveCommandData.xSpeed -= currentSpeeds.vxMetersPerSecond;
      driveCommandData.ySpeed -= currentSpeeds.vyMetersPerSecond;
      driveCommandData.xSpeed *= 1 / percentOver;
      driveCommandData.ySpeed *= 1 / percentOver;
      driveCommandData.xSpeed += currentSpeeds.vxMetersPerSecond;
      driveCommandData.ySpeed += currentSpeeds.vyMetersPerSecond;
    }

    RobotContainer.drivetrain.drive(driveCommandData);

    Logger.recordOutput("currentSpeeds", currentSpeeds);
    Logger.recordOutput("driveCommandDataSpeeds", commandedSpeeds);
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
