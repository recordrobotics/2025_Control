package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriveCommandData;

public class MoveToPoint extends Command {

    private final Drivetrain _drivetrain;

    private final Pose2d target;

    private PIDController anglePID;

    private final double maxSpeed;
    private final double slowdownDistance;
    private final double isFinishedThreshold;

    public MoveToPoint(Drivetrain drivetrain, Pose2d target) {
        this(drivetrain, target, 0.1);
    }

    public MoveToPoint(Drivetrain drivetrain, Pose2d target, double maxSpeed) {
        this(drivetrain, target, maxSpeed, 0.1);
    }

    public MoveToPoint(Drivetrain drivetrain, Pose2d target, double maxSpeed, double slowdownDistance) {
        this(drivetrain, target, maxSpeed, slowdownDistance, 0.05);
    }

    public MoveToPoint(Drivetrain drivetrain, Pose2d target, double maxSpeed, double slowdownDistance,
            double isFinishedThreshold) {
        addRequirements(drivetrain);
        setSubsystem(drivetrain.getName());
        _drivetrain = drivetrain;
        this.target = target;
        this.maxSpeed = maxSpeed;
        this.slowdownDistance = slowdownDistance;
        this.isFinishedThreshold = isFinishedThreshold;

        // Sets up PID Controller
        anglePID = new PIDController(0.4, 0, 0);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void initialize() {

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

        double spin = Math.max(-0.5,
                Math.min(0.5, anglePID.calculate(pose.getRotation().getRadians(), target.getRotation().getRadians())));

        // Gets information needed to drive
        DriveCommandData driveCommandData = new DriveCommandData(
                x_speed, // x_speed,
                y_speed, // y_speed,
                spin,
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
