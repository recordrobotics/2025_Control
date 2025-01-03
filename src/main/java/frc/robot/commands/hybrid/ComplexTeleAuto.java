package frc.robot.commands.hybrid;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.MoveToPoint;
import frc.robot.commands.auto.StopAndWait;
import frc.robot.subsystems.Drivetrain;

public class ComplexTeleAuto extends SequentialCommandGroup {

    private Drivetrain _drivetrain;
    
    public ComplexTeleAuto(Drivetrain drivetrain) {
        this._drivetrain = drivetrain;

        Translation2d current_pose = _drivetrain.poseFilter.getEstimatedPosition().getTranslation();

        addCommands(

                new MoveToPoint(_drivetrain, new Pose2d(new Translation2d(0, 1).plus(current_pose), new Rotation2d(0)),
                        0.15),
                new StopAndWait(_drivetrain, 0.2),

                new MoveToPoint(_drivetrain, new Pose2d(new Translation2d(1, 1).plus(current_pose), new Rotation2d(0)),
                        0.15),
                new StopAndWait(_drivetrain, 0.2),

                new MoveToPoint(_drivetrain, new Pose2d(new Translation2d(1, 0).plus(current_pose), new Rotation2d(0)),
                        0.15),
                new StopAndWait(_drivetrain, 0.2),

                new MoveToPoint(_drivetrain, new Pose2d(new Translation2d(0, 0).plus(current_pose), new Rotation2d(0)),
                        0.15),
                new StopAndWait(_drivetrain, 0.05));
    }
}