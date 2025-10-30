package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoScore;
import frc.robot.utils.AutoUtils;
import frc.robot.utils.DriverStationUtils;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class BargeLeftAuto extends SequentialCommandGroup implements IAutoRoutine {

    public BargeLeftAuto() {
        try {
            addCommands(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeLeftToReefJ")),
                    Commands.deferredProxy(() -> new AutoScore(
                            DriverStationUtils.getCurrentAlliance() == Alliance.Blue
                                    ? CoralPosition.BLUE_J
                                    : CoralPosition.RED_J,
                            CoralLevel.L4,
                            false)),
                    AutoUtils.createSource("J", "Left"),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefK")),
                    Commands.deferredProxy(() -> new AutoScore(
                            DriverStationUtils.getCurrentAlliance() == Alliance.Blue
                                    ? CoralPosition.BLUE_K
                                    : CoralPosition.RED_K,
                            CoralLevel.L4,
                            false)),
                    AutoUtils.createSource("K", "Left"),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("ElevatorStartToReefL")),
                    Commands.deferredProxy(() -> new AutoScore(
                            DriverStationUtils.getCurrentAlliance() == Alliance.Blue
                                    ? CoralPosition.BLUE_L
                                    : CoralPosition.RED_L,
                            CoralLevel.L4,
                            false)),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("ReefLToPark")));
            addRequirements(RobotContainer.drivetrain);
        } catch (FileVersionException | IOException | ParseException e) {
            throw new CreateAutoRoutineException("Failed to create BargeLeftAuto: " + e.getMessage(), e);
        }
    }

    @Override
    public String getAutoName() {
        return "CMD_BargeLeftOuter";
    }
}
