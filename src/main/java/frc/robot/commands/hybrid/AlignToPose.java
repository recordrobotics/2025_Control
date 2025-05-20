package frc.robot.commands.hybrid;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.assists.DrivetrainControl;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AlignToPose extends Command {

  private static DrivetrainControl drivetrainControl = null;

  private double tP = 4;

  ProfiledPIDController rotPID = // radians
      new ProfiledPIDController(
          4,
          0,
          0.05,
          new TrapezoidProfile.Constraints(
              Constants.Align.MAX_ANGULAR_VELOCITY, Constants.Align.MAX_ANGULAR_ACCELERATION));

  private Supplier<Pose2d> targetPose;

  public AlignToPose(Supplier<Pose2d> pose) {
    this.targetPose = pose;

    var p = pose.get();

    rotPID.setTolerance(Constants.Align.rotationalTolerance);
    if (p != null) {
      rotPID.setGoal(p.getRotation().getRadians());
    }
    rotPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(RobotContainer.drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d pose = RobotContainer.poseSensorFusion.getEstimatedPosition();

    rotPID.reset(pose.getRotation().getRadians());

    var p = targetPose.get();
    if (p != null) {
      rotPID.setGoal(p.getRotation().getRadians());
    }
  }

  private boolean isTAtGoal() {
    Pose2d pose = RobotContainer.poseSensorFusion.getEstimatedPosition();
    var p = targetPose.get();
    return pose.getTranslation().getDistance(p.getTranslation())
        <= Constants.Align.translationalTolerance;
  }

  @Override
  public boolean isFinished() {
    return isTAtGoal() && rotPID.atGoal();
  }

  @Override
  public void execute() {
    Pose2d pose = RobotContainer.poseSensorFusion.getEstimatedPosition();

    var p = targetPose.get();

    double x = 0;
    double y = 0;
    double rot = 0;

    if (p != null) {
      var targetVector = p.getTranslation().minus(pose.getTranslation());
      var targetDirection = targetVector.div(targetVector.getNorm());
      targetDirection =
          targetDirection.times(
              Math.min(Constants.Align.MAX_VELOCITY, tP * targetVector.getNorm()));

      var chassisSpeeds =
          ChassisSpeeds.fromRobotRelativeSpeeds(
              RobotContainer.drivetrain.getChassisSpeeds(), pose.getRotation());
      var currentVelocity =
          new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

      currentVelocity =
          MathUtil.slewRateLimit(
              currentVelocity, targetDirection, 0.02, Constants.Align.MAX_ACCELERATION);

      Logger.recordOutput("AlignToPose/Velocity", targetDirection.getNorm());
      Logger.recordOutput("AlignToPose/VelocityLimited", currentVelocity.getNorm());

      x = currentVelocity.getX();
      y = currentVelocity.getY();
      rot = rotPID.calculate(pose.getRotation().getRadians(), p.getRotation().getRadians());
    }

    Logger.recordOutput("AlignToPose/Target", p);
    Logger.recordOutput("AlignToPose/AtGoal/T", isTAtGoal());
    Logger.recordOutput("AlignToPose/AtGoal/Rot", rotPID.atGoal());

    drivetrainControl =
        DrivetrainControl.createFieldRelative(
            new Transform2d(x, y, new Rotation2d(rot)),
            Transform2d.kZero,
            Transform2d.kZero,
            RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation());

    RobotContainer.drivetrain.drive(drivetrainControl.toChassisSpeeds());
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new ChassisSpeeds());
  }

  public static DrivetrainControl getDrivetrainControl() {
    return drivetrainControl;
  }
}
