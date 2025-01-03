package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.ShuffleboardUI;
import frc.robot.utils.AutoPath;

public class PlannedAuto extends SequentialCommandGroup {
  public PlannedAuto(AutoPath autoPath) {
    addCommands(
        new InstantCommand(() -> RobotContainer.poseTracker.resetStartingPose()),
        new PathPlannerAuto(ShuffleboardUI.Autonomous.getAutoChooser()),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        new WaitCommand(0.1),
        new InstantCommand(() -> System.out.println("Auto End")));
  }
}
