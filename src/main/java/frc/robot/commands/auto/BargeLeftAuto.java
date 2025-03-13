package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Align;
import frc.robot.commands.CoralIntakeFromSource;
import frc.robot.commands.CoralShoot;
import java.io.IOException;
import java.util.Set;
import java.util.stream.Collectors;
import org.json.simple.parser.ParseException;

public class BargeLeftAuto extends SequentialCommandGroup {

  public class ScheduleCommand2 extends Command {
    private final Set<Command> m_toSchedule;

    private boolean finished = false;

    /**
     * Creates a new ScheduleCommand that schedules the given commands when initialized.
     *
     * @param toSchedule the commands to schedule
     */
    public ScheduleCommand2(Command... toSchedule) {
      m_toSchedule =
          Set.of(toSchedule).stream()
              .map(cmd -> cmd.finallyDo(() -> finished = true))
              .collect(Collectors.toSet());
    }

    @Override
    public void initialize() {
      for (Command command : m_toSchedule) {
        command.schedule();
      }
    }

    @Override
    public boolean isFinished() {
      return finished;
    }

    @Override
    public void end(boolean interrupted) {}
  }

  public BargeLeftAuto() throws FileVersionException, IOException, ParseException {
    addCommands(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeLeftToL4")).asProxy(),
        Align.createWithTimeout(0.01, 0.05, true, 1.0).asProxy(),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        new ScheduleCommand2(new CoralShoot().asProxy().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefJToSourceLeftOuter")).asProxy(),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        new ScheduleCommand2(new CoralIntakeFromSource().asProxy().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("SourceLeftOuterToReefK")).asProxy(),
        Align.createWithTimeout(0.01, 0.05, true, 1.0).asProxy(),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        new ScheduleCommand2(new CoralShoot().asProxy().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefKToSourceLeftOuter")).asProxy(),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        new ScheduleCommand2(new CoralIntakeFromSource().asProxy().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("SourceLeftOuterToReefL")).asProxy(),
        Align.createWithTimeout(0.01, 0.05, true, 1.0).asProxy(),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
        new ScheduleCommand2(new CoralShoot().asProxy().withTimeout(1.0)),
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefLToPark")).asProxy(),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()));
    addRequirements(RobotContainer.drivetrain);
  }
}
