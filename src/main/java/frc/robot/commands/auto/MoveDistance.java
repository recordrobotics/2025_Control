package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriveCommandData;

public class MoveDistance extends Command {

    private final Drivetrain _drivetrain;

    private Translation2d target;
    private final Translation2d distance;

    private final double maxSpeed;
    private final double slowdownDistance;
    private final double isFinishedThreshold;

    public MoveDistance(Drivetrain drivetrain, Translation2d distance) {
        this(drivetrain, distance, 0.1);
    }

    public MoveDistance(Drivetrain drivetrain, Translation2d distance, double maxSpeed) {
        this(drivetrain, distance, maxSpeed, 0.1);
    }

    public MoveDistance(Drivetrain drivetrain, Translation2d distance, double maxSpeed, double slowdownDistance) {
        this(drivetrain, distance, maxSpeed, slowdownDistance, 0.05);
    }

    public MoveDistance(Drivetrain drivetrain, Translation2d distance, double maxSpeed, double slowdownDistance,
            double isFinishedThreshold) {
        addRequirements(drivetrain);
        setSubsystem(drivetrain.getName());
        _drivetrain = drivetrain;
        this.distance = distance;
        this.maxSpeed = maxSpeed;
        this.slowdownDistance = slowdownDistance;
        this.isFinishedThreshold = isFinishedThreshold;
    }

    @Override
    public void initialize() {
        Pose2d pose = _drivetrain.poseFilter.getEstimatedPosition();
        target = pose.getTranslation().plus(distance);
    }

    @Override
    public void execute() {
        Pose2d pose = _drivetrain.poseFilter.getEstimatedPosition();
        Translation2d swerve_position = pose.getTranslation();

        double x_diff = target.getX() - swerve_position.getX();
        double y_diff = target.getY() - swerve_position.getY();

        // Calculates magnitude
        double magnitude = Math.sqrt(x_diff * x_diff + y_diff * y_diff);
        double clamped_magnitude = Math.min(1, magnitude / slowdownDistance);

        // Calculates x and y speeds from magnitude and diff
        double x_speed = x_diff / magnitude * maxSpeed * clamped_magnitude;
        double y_speed = y_diff / magnitude * maxSpeed * clamped_magnitude;

        // Gets information needed to drive
        DriveCommandData driveCommandData = new DriveCommandData(
                x_speed, // x_speed,
                y_speed, // y_speed,
                0,
                true);

        _drivetrain.drive(driveCommandData);
    }

    @Override
    public void end(boolean interrupted) {
        _drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = _drivetrain.poseFilter.getEstimatedPosition();
        Translation2d swerve_position = pose.getTranslation();

        double x_diff = target.getX() - swerve_position.getX();
        double y_diff = target.getY() - swerve_position.getY();

        // Calculates magnitude
        double magnitude = Math.sqrt(x_diff * x_diff + y_diff * y_diff);

        return magnitude < isFinishedThreshold;
    }

}
