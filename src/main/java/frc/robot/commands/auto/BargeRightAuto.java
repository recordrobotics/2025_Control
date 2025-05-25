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

public class BargeRightAuto extends SequentialCommandGroup {
  public BargeRightAuto() throws FileVersionException, IOException, ParseException {
    addCommands(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeRightToL4")),
        CommandUtils.finishOnInterrupt(AutoUtils.alignWithVision().withTimeout(1.5)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        AutoUtils.createSource("E", "Right"),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefD")),
        CommandUtils.finishOnInterrupt(AutoUtils.alignWithVision().withTimeout(1.5)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        AutoUtils.createSource("D", "Right"),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefC")),
        CommandUtils.finishOnInterrupt(AutoUtils.alignWithVision().withTimeout(1.5)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefCToPark")),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()));
    addRequirements(RobotContainer.drivetrain);
  }
}
