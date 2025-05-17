package frc.robot.commands.hybrid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.assists.DrivetrainControl;
import org.littletonrobotics.junction.Logger;

public class AlignToPose extends Command {
  ProfiledPIDController xPID =
      new ProfiledPIDController( // meters
          9,
          0,
          0.01,
          new TrapezoidProfile.Constraints(
              Constants.Align.MAX_VELOCITY, Constants.Align.MAX_ACCELERATION));
  ProfiledPIDController yPID = // meters
      new ProfiledPIDController(
          9,
          0,
          0.01,
          new TrapezoidProfile.Constraints(
              Constants.Align.MAX_VELOCITY, Constants.Align.MAX_ACCELERATION));
  ProfiledPIDController rotPID = // radians
      new ProfiledPIDController(
          4,
          0,
          0.05,
          new TrapezoidProfile.Constraints(
              Constants.Align.MAX_ANGULAR_VELOCITY, Constants.Align.MAX_ANGULAR_ACCELERATION));
  boolean doTranslation;

  private Pose2d targetPose;

  public AlignToPose(Pose2d pose, boolean doTranslation) {
    this.targetPose = pose;

    if (doTranslation) {
      xPID.setTolerance(Constants.Align.translationalTolerance);
      yPID.setTolerance(Constants.Align.translationalTolerance);
      xPID.setGoal(pose.getX());
      yPID.setGoal(pose.getY());
    }
    rotPID.setTolerance(Constants.Align.rotationalTolerance);
    rotPID.setGoal(pose.getRotation().getRadians());
    rotPID.enableContinuousInput(-Math.PI, Math.PI);

    this.doTranslation = doTranslation;

    addRequirements(RobotContainer.drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d pose = RobotContainer.poseSensorFusion.getEstimatedPosition();
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
    Pose2d pose = RobotContainer.poseSensorFusion.getEstimatedPosition();

    double x = xPID.calculate(pose.getX());
    double y = yPID.calculate(pose.getY());
    double rot = rotPID.calculate(pose.getRotation().getRadians());

    Logger.recordOutput("AlignToPose/Target", targetPose);
    Logger.recordOutput("AlignToPose/AtGoal/X", xPID.atGoal());
    Logger.recordOutput("AlignToPose/AtGoal/Y", yPID.atGoal());
    Logger.recordOutput("AlignToPose/AtGoal/Rot", rotPID.atGoal());

    AbstractControl controls = DashboardUI.Overview.getControl();
    DrivetrainControl drivetrainControl = controls.getDrivetrainControl();

    if (doTranslation) {
      drivetrainControl.applyWeightedVelocity(
          DrivetrainControl.fieldToRobot(
              new Transform2d(x, y, new Rotation2d(rot)),
              RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation()),
          1.0);
    } else {
      drivetrainControl.applyWeightedVelocity(
          new Transform2d(
              drivetrainControl.getTargetVelocity().getTranslation(), new Rotation2d(rot)),
          1.0);
    }

    RobotContainer.drivetrain.drive(drivetrainControl.toChassisSpeeds());
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new ChassisSpeeds());
  }
}
