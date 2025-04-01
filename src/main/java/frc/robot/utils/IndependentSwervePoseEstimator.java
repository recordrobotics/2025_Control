package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveModule;

public class IndependentSwervePoseEstimator {

  private final IndependentSwerveModuleState[] modules;

  private Pose2d robotPose;

  public IndependentSwervePoseEstimator(
      Pose2d initialRobotEstimate, SwerveModule[] modules, Translation2d[] moduleRobotPositions) {
    if (modules.length != moduleRobotPositions.length) {
      throw new IllegalArgumentException(
          "Modules and moduleRobotPositions must be the same length");
    }

    this.modules = new IndependentSwerveModuleState[modules.length];

    for (int i = 0; i < modules.length; i++) {
      this.modules[i] =
          new IndependentSwerveModuleState(
              initialRobotEstimate.getRotation(),
              modules[i],
              moduleRobotPositions[i],
              moduleRobotPositions[i]
                  .plus(initialRobotEstimate.getTranslation())
                  .rotateAround(
                      initialRobotEstimate.getTranslation(), initialRobotEstimate.getRotation()));
    }

    robotPose = initialRobotEstimate;
  }

  public void update(Rotation2d gyroAngle) {
    // update individual modules
    for (IndependentSwerveModuleState module : modules) {
      module.update(gyroAngle);
    }

    // use two closest approximations to correct other two
    Pose2d[] estimatedRobotPoses = getEstimatedRobotPoses();
    PoseDistance closest = null;

    for (int i = 0; i < estimatedRobotPoses.length; i++) {
      for (int j = 0; j < estimatedRobotPoses.length; j++) {
        if (i != j) {
          double distance =
              estimatedRobotPoses[i]
                  .getTranslation()
                  .getDistance(estimatedRobotPoses[j].getTranslation());
          if (closest == null || distance < closest.getDistance()) {
            closest = new PoseDistance(i, j, distance);
          }
        }
      }
    }

    if (closest != null) {
      robotPose =
          estimatedRobotPoses[closest.getPoseA()].interpolate(
              estimatedRobotPoses[closest.getPoseB()], 0.5);

      for (int i = 0; i < modules.length; i++) {
        if (i != closest.getPoseA() && i != closest.getPoseB()) {
          modules[i].reset(
              robotPose.getRotation(),
              modules[i]
                  .getModuleRobotPosition()
                  .plus(robotPose.getTranslation())
                  .rotateAround(robotPose.getTranslation(), robotPose.getRotation()));
        }
      }

    } else if (estimatedRobotPoses.length > 0) {
      robotPose = estimatedRobotPoses[0];
    }
  }

  public void reset(Pose2d robotEstimate) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].reset(
          robotEstimate.getRotation(),
          modules[i]
              .getModuleRobotPosition()
              .plus(robotEstimate.getTranslation())
              .rotateAround(robotEstimate.getTranslation(), robotEstimate.getRotation()));
    }
    robotPose = robotEstimate;
  }

  public Pose2d[] getEstimatedModulePositions() {
    Pose2d[] positions = new Pose2d[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].position;
    }
    return positions;
  }

  public Pose2d[] getEstimatedRobotPoses() {
    Pose2d[] poses = new Pose2d[modules.length];
    for (int i = 0; i < modules.length; i++) {
      poses[i] = modules[i].getEstimatedRobotPose();
    }
    return poses;
  }

  public Pose2d getEstimatedRobotPose() {
    return robotPose;
  }

  private static final class PoseDistance {
    private final int poseA;
    private final int poseB;
    private final double distance;

    public PoseDistance(int poseA, int poseB, double distance) {
      this.poseA = poseA;
      this.poseB = poseB;
      this.distance = distance;
    }

    public int getPoseA() {
      return poseA;
    }

    public int getPoseB() {
      return poseB;
    }

    public double getDistance() {
      return distance;
    }
  }

  public static final class IndependentSwerveModuleState {
    private Pose2d position;
    private Transform2d velocity;

    private Translation2d moduleRobotPosition;
    private SwerveModule module;

    private double lastUpdateTime;

    private Pose2d estimatedRobotPose = new Pose2d();

    public IndependentSwerveModuleState(
        Rotation2d gyroAngle,
        SwerveModule module,
        Translation2d moduleRobotPosition,
        Translation2d position) {
      this.module = module;
      this.moduleRobotPosition = moduleRobotPosition;
      this.position = new Pose2d(position, module.getTurnWheelRotation2d().plus(gyroAngle));
      this.velocity = calculateVelocity(gyroAngle, module);
      lastUpdateTime = Timer.getTimestamp();
    }

    public IndependentSwerveModuleState(Pose2d position, Transform2d velocity) {
      this.position = position;
      this.velocity = velocity;
      lastUpdateTime = Timer.getTimestamp();
    }

    public void reset(Pose2d position, Transform2d velocity) {
      this.position = position;
      this.velocity = velocity;
      lastUpdateTime = Timer.getTimestamp();
    }

    public void reset(Rotation2d gyroAngle, Translation2d position) {
      reset(
          new Pose2d(position, module.getTurnWheelRotation2d().plus(gyroAngle)),
          calculateVelocity(gyroAngle, module));
    }

    public void update(Rotation2d gyroAngle, SwerveModule module) {
      this.module = module;
      update(gyroAngle);
    }

    public void update(Rotation2d gyroAngle) {
      velocity = calculateVelocity(gyroAngle, module);

      double dt = MathSharedStore.getTimestamp() - lastUpdateTime;

      position =
          new Pose2d(
              new Translation2d(
                  position.getX() + velocity.getX() * dt, position.getY() + velocity.getY() * dt),
              module.getTurnWheelRotation2d().plus(gyroAngle));

      estimatedRobotPose = estimateRobotPose(gyroAngle);

      lastUpdateTime = MathSharedStore.getTimestamp();
    }

    public Translation2d getModuleRobotPosition() {
      return moduleRobotPosition;
    }

    public Pose2d getEstimatedRobotPose() {
      return estimatedRobotPose;
    }

    private static Transform2d calculateVelocity(Rotation2d gyroAngle, SwerveModule module) {
      double linearSpeed = module.getDriveWheelVelocity();
      Rotation2d heading = module.getTurnWheelRotation2d().plus(gyroAngle);

      Translation2d fieldTranslation = new Translation2d(linearSpeed, 0).rotateBy(heading);

      return new Transform2d(
          fieldTranslation.getX(),
          fieldTranslation.getY(),
          Rotation2d.fromRotations(module.getTurnWheelVelocity()));
    }

    private Pose2d estimateRobotPose(Rotation2d gyroAngle) {
      Translation2d robotOrigin =
          position
              .getTranslation()
              .minus(moduleRobotPosition)
              .rotateAround(position.getTranslation(), gyroAngle);
      return new Pose2d(robotOrigin, gyroAngle);
    }
  }
}
