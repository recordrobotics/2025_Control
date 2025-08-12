package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveModule;

public class IndependentSwervePoseEstimator {

    private final IndependentSwerveModuleState[] modules;

    private Pose2d robotPose;

    public IndependentSwervePoseEstimator(
            Pose2d initialRobotEstimate, SwerveModule[] modules, Translation2d[] moduleRobotPositions) {
        if (modules.length != moduleRobotPositions.length) {
            throw new IllegalArgumentException("Modules and moduleRobotPositions must be the same length");
        }

        this.modules = new IndependentSwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            this.modules[i] = new IndependentSwerveModuleState(
                    initialRobotEstimate.getRotation(),
                    modules[i],
                    moduleRobotPositions[i],
                    moduleRobotPositions[i]
                            .plus(initialRobotEstimate.getTranslation())
                            .rotateAround(initialRobotEstimate.getTranslation(), initialRobotEstimate.getRotation()));
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
                    double distance = estimatedRobotPoses[i]
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
                    estimatedRobotPoses[closest.getPoseA()].interpolate(estimatedRobotPoses[closest.getPoseB()], 0.5);

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

        private Translation2d moduleRobotPosition;
        private SwerveModule module;

        private Pose2d estimatedRobotPose = new Pose2d();

        private double lastDistance = 0;
        private Rotation2d lastAngle = Rotation2d.kZero;

        public IndependentSwerveModuleState(
                Rotation2d gyroAngle, SwerveModule module, Translation2d moduleRobotPosition, Translation2d position) {
            this.module = module;
            this.moduleRobotPosition = moduleRobotPosition;
            this.lastDistance = module.getDriveWheelDistance();
            this.lastAngle = module.getTurnWheelRotation2d().plus(gyroAngle).plus(Rotation2d.kCW_90deg);
            this.position = new Pose2d(position, lastAngle);
        }

        public IndependentSwerveModuleState(Pose2d position, double distance, Rotation2d angle) {
            this.position = position;
            this.lastDistance = distance;
            this.lastAngle = angle;
        }

        public void reset(Pose2d position, double distance, Rotation2d angle) {
            this.position = position;
            this.lastDistance = distance;
            this.lastAngle = angle;
        }

        public void reset(Rotation2d gyroAngle, Translation2d position) {
            double distance = module.getDriveWheelDistance();
            Rotation2d angle = module.getTurnWheelRotation2d().plus(gyroAngle).plus(Rotation2d.kCW_90deg);

            reset(new Pose2d(position, angle), distance, angle);
        }

        public void update(Rotation2d gyroAngle, SwerveModule module) {
            this.module = module;
            update(gyroAngle);
        }

        public void update(Rotation2d gyroAngle) {
            double newDistance = module.getDriveWheelDistance();
            double distanceTraveled = newDistance - lastDistance;
            lastDistance = newDistance;

            Rotation2d newAngle =
                    module.getTurnWheelRotation2d().plus(gyroAngle).plus(Rotation2d.kCW_90deg);

            Translation2d movement;
            if (lastAngle.equals(newAngle)) {
                // moved in straight line
                movement =
                        new Translation2d(distanceTraveled * newAngle.getCos(), distanceTraveled * newAngle.getSin());
            } else {
                double turnRadius = distanceTraveled / (newAngle.getRadians() - lastAngle.getRadians());
                movement = new Translation2d(
                        turnRadius * (newAngle.getCos() - lastAngle.getCos()),
                        turnRadius * (newAngle.getSin() - lastAngle.getSin()));
            }

            lastAngle = newAngle;

            position = new Pose2d(position.getTranslation().plus(movement), newAngle);

            estimatedRobotPose = estimateRobotPose(gyroAngle);
        }

        public Translation2d getModuleRobotPosition() {
            return moduleRobotPosition;
        }

        public Pose2d getEstimatedRobotPose() {
            return estimatedRobotPose;
        }

        private Pose2d estimateRobotPose(Rotation2d gyroAngle) {
            Translation2d robotOrigin = position.getTranslation()
                    .minus(moduleRobotPosition)
                    .rotateAround(position.getTranslation(), gyroAngle);
            return new Pose2d(robotOrigin, gyroAngle);
        }
    }
}
