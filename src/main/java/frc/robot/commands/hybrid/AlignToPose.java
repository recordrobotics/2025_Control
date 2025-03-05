package frc.robot.commands.hybrid;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.DriveCommandData;

public class AlignToPose extends Command {
  PIDController xPID = new PIDController(1, 0, 0);
  PIDController yPID = new PIDController(1, 0, 0);
  PIDController rotPID = new PIDController(0.1, 0, 0);
  boolean doTranslation;

  public AlignToPose(Pose2d pose, double tolerance, double rotTol, boolean doTranslation) {
    if (doTranslation) {
      xPID.setTolerance(tolerance);
      yPID.setTolerance(tolerance);
      xPID.setSetpoint(pose.getX());
      yPID.setSetpoint(pose.getY());
    }
    rotPID.setTolerance(rotTol);
    rotPID.setSetpoint(pose.getRotation().getRadians());
    rotPID.enableContinuousInput(-Math.PI, Math.PI);

    this.doTranslation = doTranslation;

    addRequirements(RobotContainer.drivetrain);
  }

  @Override
  public boolean isFinished() {
    if (!doTranslation) return rotPID.atSetpoint();
    return xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint();
  }

  @Override
  public void execute() {
    Pose2d pose = RobotContainer.poseTracker.getEstimatedPosition();
    if (doTranslation) {
      RobotContainer.drivetrain.drive(
          new DriveCommandData(
              xPID.calculate(pose.getX()),
              yPID.calculate(pose.getY()),
              rotPID.calculate(pose.getRotation().getRadians()),
              true));
    } else {
      AbstractControl controls = DashboardUI.Overview.getControl();
      DriveCommandData driveCommandData = controls.getDriveCommandData();

      RobotContainer.drivetrain.drive(
          new DriveCommandData(
              driveCommandData.xSpeed,
              driveCommandData.ySpeed,
              rotPID.calculate(pose.getRotation().getRadians()),
              driveCommandData.fieldRelative));
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new DriveCommandData());
  }
}
