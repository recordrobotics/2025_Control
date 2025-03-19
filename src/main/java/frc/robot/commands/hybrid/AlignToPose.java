package frc.robot.commands.hybrid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.DriveCommandData;
import org.littletonrobotics.junction.Logger;

public class AlignToPose extends Command {
  ProfiledPIDController xPID =
      new ProfiledPIDController(
          4,
          0,
          0.01,
          new TrapezoidProfile.Constraints(
              Constants.Swerve.MAX_AUTOALIGN_VELOCITY,
              Constants.Swerve.MAX_AUTOALIGN_ACCELERATION));
  ProfiledPIDController yPID =
      new ProfiledPIDController(
          4,
          0,
          0.01,
          new TrapezoidProfile.Constraints(
              Constants.Swerve.MAX_AUTOALIGN_VELOCITY,
              Constants.Swerve.MAX_AUTOALIGN_ACCELERATION));
  ProfiledPIDController rotPID =
      new ProfiledPIDController(
          4,
          0,
          0.05,
          new TrapezoidProfile.Constraints(
              Constants.Swerve.MAX_AUTOALIGN_ANGULAR_VELOCITY,
              Constants.Swerve.MAX_AUTOALIGN_ANGULAR_ACCELERATION));
  boolean doTranslation;

  private Pose2d targetPose;

  public AlignToPose(Pose2d pose, double tolerance, double rotTol, boolean doTranslation) {
    this.targetPose = pose;

    if (doTranslation) {
      xPID.setTolerance(tolerance);
      yPID.setTolerance(tolerance);
      xPID.setGoal(pose.getX());
      yPID.setGoal(pose.getY());
    }
    rotPID.setTolerance(rotTol);
    rotPID.setGoal(pose.getRotation().getRadians());
    rotPID.enableContinuousInput(-Math.PI, Math.PI);

    this.doTranslation = doTranslation;

    addRequirements(RobotContainer.drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d pose = RobotContainer.poseTracker.getEstimatedPosition();
    xPID.reset(pose.getX());
    yPID.reset(pose.getY());
    rotPID.reset(pose.getRotation().getRadians());
  }

  @Override
  public boolean isFinished() {
    if (!doTranslation) return false;
    return xPID.atGoal() && yPID.atGoal() && rotPID.atGoal();
  }

  @Override
  public void execute() {
    Pose2d pose = RobotContainer.poseTracker.getEstimatedPosition();

    double x = xPID.calculate(pose.getX());
    double y = yPID.calculate(pose.getY());
    double rot = rotPID.calculate(pose.getRotation().getRadians());

    Logger.recordOutput("AlignToPose/Target", targetPose);

    if (doTranslation) {
      RobotContainer.drivetrain.drive(new DriveCommandData(x, y, rot, true));
    } else {
      AbstractControl controls = DashboardUI.Overview.getControl();
      DriveCommandData driveCommandData = controls.getDriveCommandData();

      RobotContainer.drivetrain.drive(
          new DriveCommandData(
              driveCommandData.xSpeed,
              driveCommandData.ySpeed,
              rot,
              driveCommandData.fieldRelative));
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new DriveCommandData());
  }
}
