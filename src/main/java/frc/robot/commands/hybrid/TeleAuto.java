package frc.robot.commands.hybrid;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriveCommandData;


public class TeleAuto extends Command {

    private final Drivetrain _drivetrain;
    private final DoubleControl _controls;

    private final Pose2d target;

    private final double slowdownDistance = 0.1;
    private final double maxSpeed = 0.1;
    private double isFinishedThreshold = 0.05;

    private PIDController anglePID;

    public TeleAuto(Drivetrain drivetrain, DoubleControl controls) {
        addRequirements(drivetrain);
        setSubsystem(drivetrain.getName());
        _drivetrain = drivetrain;
        _controls = controls;

        // Init target
        Pose2d current_pose = _drivetrain.poseFilter.getEstimatedPosition();
        Transform2d transform_pose = new Transform2d(new Translation2d(1, 1), new Rotation2d(0));
        target = current_pose.plus(transform_pose);

        // Sets up PID Controller
        anglePID = new PIDController(0.4, 0, 0);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        _drivetrain.stop();
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

        // If the auto kill switch is pressed, returns true
        if (_controls.getKillAuto()) {
            return true;
        }

        Pose2d pose = _drivetrain.poseFilter.getEstimatedPosition();
        Translation2d swerve_position = pose.getTranslation();

        double x_diff = target.getX() - swerve_position.getX();
        double y_diff = target.getY() - swerve_position.getY();

        // Calculates magnitude
        double magnitude = Math.sqrt(x_diff * x_diff + y_diff * y_diff);

        // Returns
        return magnitude < isFinishedThreshold;
    }

}
