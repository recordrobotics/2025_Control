package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Auto;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.Constants.Game.CoralPosition;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.RobotContainer;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.WaypointAlign;
import frc.robot.commands.legacy.CoralIntakeFromSource;
import frc.robot.subsystems.ElevatorHead.GamePiece;
import java.io.IOException;
import java.util.Set;
import org.json.simple.parser.ParseException;

public final class AutoUtils {
    private AutoUtils() {}

    public static Command alignWithVision() {
        return Commands.defer(
                () -> {
                    int[] visionTagTarget = {
                        IGamePosition.closestTo(
                                        RobotContainer.poseSensorFusion.getEstimatedPosition(), CoralPosition.values())
                                .apriltagId
                    };

                    // TODO: Add 2d april tag alignment after waypoint finishes

                    return WaypointAlign.align(
                            ReefAlign.generateWaypointsClosestWithOffset(CoralLevel.L4, false),
                            0,
                            1,
                            true,
                            new Double[] {2.0, 1.0});
                },
                Set.of(RobotContainer.drivetrain));
    }

    // How long to wait after starting drive to source before deploying intake
    public static final double SOURCE_INTAKE_DEPLOY_DELAY = 0.3;
    // If the robot is within this distance of the closest reef, it will drive to source from there, otherwise go from
    // ElevatorStart waypoint
    public static final double REEF_DISTANCE_THRESHOLD = 0.7;

    public static Command createSource(String reefLetter, String sourceSide)
            throws FileVersionException, IOException, ParseException {
        return new CoralIntakeFromSource(false)
                .beforeStarting(new WaitCommand(SOURCE_INTAKE_DEPLOY_DELAY))
                .alongWith(Commands.either(
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                                                "Reef" + reefLetter + "ToSource" + sourceSide + "OuterNoElevator"))
                                        .andThen(new WaitCommand(Auto.SOURCE_TIMEOUT))
                                        .andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                                                "Source" + sourceSide + "OuterToElevatorStart"))),
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                                                "ElevatorStartToSource" + sourceSide + "Outer"))
                                        .andThen(new WaitCommand(Auto.SOURCE_TIMEOUT))
                                        .andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                                                "Source" + sourceSide + "OuterToElevatorStart"))),
                                () -> IGamePosition.closestTo(
                                                        RobotContainer.poseSensorFusion.getEstimatedPosition(),
                                                        CoralPosition.values())
                                                .getFirstStagePose()
                                                .getTranslation()
                                                .getDistance(RobotContainer.poseSensorFusion
                                                        .getEstimatedPosition()
                                                        .getTranslation())
                                        < REEF_DISTANCE_THRESHOLD)
                        .repeatedly()
                        .onlyWhile(() ->
                                !RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.CORAL)));
    }
}
