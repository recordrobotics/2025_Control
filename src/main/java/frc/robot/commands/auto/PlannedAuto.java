package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutoPath;

public class PlannedAuto extends SequentialCommandGroup {

    public PlannedAuto(AutoPath autoPath) {

        addCommands(
                new PathPlannerAuto("TestTeleAuto"));
    }

}