package frc.robot.commands.hybrid;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.DriveCommandData;

public class AlignToPose extends Command {
  PIDController xPID = new PIDController(1, 0, 0);
  PIDController yPID = new PIDController(1, 0, 0);
  PIDController rotPID = new PIDController(1, 0, 0);
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
    this.doTranslation = doTranslation;
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
      RobotContainer.drivetrain.drive(
          new DriveCommandData(0, 0, rotPID.calculate(pose.getRotation().getRadians()), true));
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new DriveCommandData());
  }
}
