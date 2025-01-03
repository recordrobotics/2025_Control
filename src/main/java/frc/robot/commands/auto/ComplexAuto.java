package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;


public class ComplexAuto extends SequentialCommandGroup {

    private Drivetrain _drivetrain;
    
    public ComplexAuto (Drivetrain drivetrain) {
        this._drivetrain = drivetrain;

        Translation2d current_pose = _drivetrain.poseFilter.getEstimatedPosition().getTranslation();

        addCommands(

            new MoveToPoint(_drivetrain, new Pose2d(new Translation2d(0, 1).plus(current_pose), new Rotation2d(0)), 0.15),
            new StopAndWait(_drivetrain, 0.2),

            new MoveToPoint(_drivetrain, new Pose2d(new Translation2d(1, 1).plus(current_pose), new Rotation2d(0)), 0.15),
            new StopAndWait(_drivetrain, 0.2),
            
            new MoveToPoint(_drivetrain, new Pose2d(new Translation2d(1, 0).plus(current_pose), new Rotation2d(0)), 0.15),
            new StopAndWait(_drivetrain, 0.2),

            new MoveToPoint(_drivetrain, new Pose2d(new Translation2d(0, 0).plus(current_pose), new Rotation2d(0)), 0.15),
            
            new RepeatCommand(
                new InstantCommand(()->{
                drivetrain.stop();
            }))
        );
    }

}