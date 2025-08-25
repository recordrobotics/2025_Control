package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Constants.Game.AlgaePosition;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorHead.AlgaeGrabberStates;

public class AutoAlgae extends SequentialCommandGroup {

    private static boolean cancelCommand = false;
    private static boolean isRunning = false;

    public static void performCancel() {
        cancelCommand = true;
    }

    public static boolean isRunning() {
        return isRunning;
    }

    public static void stopRunning() {
        isRunning = false;
    }

    private static void resetCancel() {
        cancelCommand = false;
    }

    private static void startRunning() {
        isRunning = true;
    }

    public static Command algaeGrabCommand(ElevatorHeight targetHeight) {
        return new ElevatorMove(targetHeight)
                .andThen(new InstantCommand(
                        () -> {
                            if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE)
                                RobotContainer.elevatorHead.set(AlgaeGrabberStates.INTAKE_GROUND);
                            else RobotContainer.elevatorHead.set(AlgaeGrabberStates.INTAKE_REEF);
                        },
                        RobotContainer.elevatorHead));
    }

    private static final double AUTO_LOWER_DISTANCE = 1.0;

    // 8s timeout for first waypoint
    private static final double FIRST_WAYPOINT_TIMEOUT = 8.0;
    // 4s for second
    private static final double OTHER_WAYPOINT_TIMEOUT = 4.0;

    public AutoAlgae(AlgaePosition reefPole) {
        addCommands(
                new InstantCommand(() -> {
                    resetCancel();
                    startRunning();
                }),
                WaypointAlign.alignWithCommand(
                                AlgaeAlign.generateWaypoints(reefPole),
                                new Double[] {FIRST_WAYPOINT_TIMEOUT, OTHER_WAYPOINT_TIMEOUT},
                                // start elevator immediately
                                -1,
                                // elevator has to be fully extended before moving to second waypoint
                                0,
                                algaeGrabCommand(reefPole.getLevel().getHeight())
                                        .asProxy())
                        .onlyWhile(() -> RobotState.isAutonomous() || !cancelCommand),
                new WaitUntilCommand(() -> {
                    Pose2d pose = reefPole.getPose();

                    double dist = RobotContainer.poseSensorFusion
                            .getEstimatedPosition()
                            .getTranslation()
                            .getDistance(pose.getTranslation());

                    return cancelCommand || (dist > AUTO_LOWER_DISTANCE);
                }),
                new ElevatorMoveThenAlgaeGrabEnd(reefPole.getLevel().getHeight(), true).asProxy());
    }
}
