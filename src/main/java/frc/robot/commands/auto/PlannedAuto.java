package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoPath;
import frc.robot.subsystems.Drivetrain;

public class PlannedAuto extends SequentialCommandGroup {
    public PlannedAuto(Drivetrain drivetrain, AutoPath autoPath) {
        addCommands(
                new InstantCommand(() -> drivetrain.resetPose()),
                new PathPlannerAuto(RobotContainer.autoChooser.getSelected().getPathRef()),
                new InstantCommand(() -> drivetrain.stop()),
                new WaitCommand(0.1),
                new InstantCommand(() -> System.out.println("ContainerAuto End"))
                );
    }
}