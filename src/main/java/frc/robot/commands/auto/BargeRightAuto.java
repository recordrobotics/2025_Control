package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Align;
import frc.robot.commands.CoralIntakeFromSource;
import frc.robot.commands.CoralShoot;
import frc.robot.utils.CommandUtils;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class BargeRightAuto extends SequentialCommandGroup {

  public BargeRightAuto() throws FileVersionException, IOException, ParseException {
    addCommands(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeRightToL4")),
        CommandUtils.finishOnInterrupt(Align.create(0.01, 0.05, true).withTimeout(1.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        new CoralIntakeFromSource(false).beforeStarting(new WaitCommand(0.2))
            .withDeadline(
                AutoBuilder.followPath(
                        PathPlannerPath.fromPathFile("ReefEToSourceRightOuterNoElevator"))
                    .andThen(new InstantCommand(() -> RobotContainer.drivetrain.kill()))
                    .andThen(new WaitCommand(0.6))
                    .andThen(
                        AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile("SourceRightOuterToElevatorStart")))),
        new CoralIntakeFromSource(false)
            .withDeadline(
                AutoBuilder.followPath(
                        PathPlannerPath.fromPathFile("ElevatorStartToSourceRightOuter"))
                    .andThen(new InstantCommand(() -> RobotContainer.drivetrain.kill()))
                    .andThen(new WaitCommand(0.6))
                    .andThen(
                        AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile("SourceRightOuterToElevatorStart"))))
            .repeatedly()
            .onlyWhile(() -> !RobotContainer.coralShooter.hasCoral()),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefD")),
        CommandUtils.finishOnInterrupt(Align.create(0.01, 0.05, true).withTimeout(1.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        new CoralIntakeFromSource(false).beforeStarting(new WaitCommand(0.2))
            .withDeadline(
                AutoBuilder.followPath(
                        PathPlannerPath.fromPathFile("ReefDToSourceRightOuterNoElevator"))
                    .andThen(new InstantCommand(() -> RobotContainer.drivetrain.kill()))
                    .andThen(new WaitCommand(0.6))
                    .andThen(
                        AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile("SourceRightOuterToElevatorStart")))),
        new CoralIntakeFromSource(false)
            .withDeadline(
                AutoBuilder.followPath(
                        PathPlannerPath.fromPathFile("ElevatorStartToSourceRightOuter"))
                    .andThen(new InstantCommand(() -> RobotContainer.drivetrain.kill()))
                    .andThen(new WaitCommand(0.6))
                    .andThen(
                        AutoBuilder.followPath(
                            PathPlannerPath.fromPathFile("SourceRightOuterToElevatorStart"))))
            .repeatedly()
            .onlyWhile(() -> !RobotContainer.coralShooter.hasCoral()),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefC")),
        CommandUtils.finishOnInterrupt(Align.create(0.01, 0.05, true).withTimeout(1.0)),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefCToPark")),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()));
    addRequirements(RobotContainer.drivetrain);
  }
}
