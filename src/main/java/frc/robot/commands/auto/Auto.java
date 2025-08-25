package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldStartingLocation;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.SourcePosition;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralShoot;
import frc.robot.utils.AutoUtils;
import frc.robot.utils.CommandUtils;
import java.io.IOException;
import java.util.List;
import java.util.stream.IntStream;
import java.util.stream.Stream;
import org.json.simple.parser.ParseException;

public class Auto {
    public Command create(
            List<Pair<CoralPosition, CoralLevel>> scoringPositions,
            FieldStartingLocation startPosition,
            SourcePosition source)
            throws FileVersionException, IOException, ParseException {
        return create(
                scoringPositions.stream()
                        .map(p -> Pair.of(
                                String.valueOf(p.getFirst()
                                        .name()
                                        .charAt(p.getFirst().name().length() - 1)),
                                Integer.parseInt(
                                        String.valueOf(p.getSecond().name().charAt(1)))))
                        .toList(),
                startPosition,
                source.name().substring(9));
    }

    /**
     * @param scoringPositions List[Pair[Position (A-L), Level(1-4)]]
     * @param startPosition FieldStartingLocation
     * @param source "Left" or "Right"
     * @return the command
     * @throws FileVersionException
     * @throws IOException
     * @throws ParseException
     */
    public Command create(
            List<Pair<String, Integer>> scoringPositions, FieldStartingLocation startPosition, String source)
            throws FileVersionException, IOException, ParseException {
        return new SequentialCommandGroup(Stream.concat(
                        Stream.of(
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile(startPosition.name() + "ToReef"
                                        + scoringPositions.get(0).getFirst())),
                                CommandUtils.finishOnInterrupt(
                                        AutoUtils.alignWithVision().withTimeout(2.0)),
                                new InstantCommand(() -> RobotContainer.drivetrain.kill()),
                                CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
                                AutoUtils.createSource(
                                        String.valueOf(scoringPositions.get(0).getFirst()), source)),
                        IntStream.rangeClosed(1, scoringPositions.size()).mapToObj(i -> {
                            try {
                                return new SequentialCommandGroup(
                                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReef"
                                                + String.valueOf(
                                                        scoringPositions.get(i).getFirst()))),
                                        CommandUtils.finishOnInterrupt(
                                                AutoUtils.alignWithVision().withTimeout(2.0)),
                                        new InstantCommand(() -> RobotContainer.drivetrain.kill()),
                                        CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(1.0)),
                                        AutoUtils.createSource(
                                                String.valueOf(
                                                        scoringPositions.get(0).getFirst()),
                                                source));
                            } catch (IOException | ParseException e) {
                                throw new RuntimeException(e);
                            }
                        }))
                .toArray(Command[]::new)) {
            {
                addRequirements(RobotContainer.drivetrain);
            }
        };
    }
}
