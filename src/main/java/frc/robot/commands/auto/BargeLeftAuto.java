package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralShoot;
import frc.robot.utils.AutoUtils;
import frc.robot.utils.CommandUtils;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class BargeLeftAuto extends SequentialCommandGroup {

  public BargeLeftAuto() throws FileVersionException, IOException, ParseException {
    addCommands(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeLeftToL4")),
        CommandUtils.finishOnInterrupt(AutoUtils.alignWithVision().withTimeout(2.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        AutoUtils.createSource("J", "Left"),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefK")),
        CommandUtils.finishOnInterrupt(AutoUtils.alignWithVision().withTimeout(2.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        AutoUtils.createSource("K", "Left"),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefL")),
        CommandUtils.finishOnInterrupt(AutoUtils.alignWithVision().withTimeout(2.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefLToPark")),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()));
    addRequirements(RobotContainer.drivetrain);
  }
}
