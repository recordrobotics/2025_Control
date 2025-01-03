package frc.robot.commands.auto;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.AutoPath;

public class PlannedAuto extends SequentialCommandGroup {
    public PlannedAuto(Drivetrain drivetrain, AutoPath autoPath) {
        addCommands(
                new InstantCommand(() -> drivetrain.resetStartingPose()),
                new PathPlannerAuto(ShuffleboardUI.Autonomous.getAutoChooser()),
                new InstantCommand(() -> drivetrain.kill()),
                new WaitCommand(0.1),
                new InstantCommand(() -> System.out.println("Auto End")));
    }
}