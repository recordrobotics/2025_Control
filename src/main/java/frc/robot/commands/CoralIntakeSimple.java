package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeToElevator;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
import frc.robot.subsystems.ElevatorHead.GamePiece;
import frc.robot.utils.CommandUtils;

public class CoralIntakeSimple extends SequentialCommandGroup {

    private static boolean running = false;

    public CoralIntakeSimple(boolean useElevatorProxy) {
        addCommands(
                new InstantCommand(
                        () -> {
                            setRunning(true);
                            RobotContainer.coralIntake.set(CoralIntakeState.SOURCE);
                        },
                        RobotContainer.coralIntake),
                // start moving elevator to intake position
                CommandUtils.maybeProxy(useElevatorProxy, new ElevatorMove(ElevatorHeight.INTAKE)),
                new InstantCommand(
                        () -> RobotContainer.elevatorHead.set(CoralShooterStates.INTAKE), RobotContainer.elevatorHead),
                new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
                // wait for elevator to have coral
                new CoralIntakeToElevator()
                        .simulateFor(new WaitUntilCommand(
                                () -> RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.CORAL))),
                new InstantCommand(
                        () -> {
                            RobotContainer.coralIntake.set(CoralIntakeState.UP);
                            // move coral a set distance
                            RobotContainer.elevatorHead.moveBy(Constants.ElevatorHead.CORAL_INTAKE_DISTANCE);
                        },
                        RobotContainer.coralIntake,
                        RobotContainer.elevatorHead),
                new WaitUntilCommand(
                        () -> RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.CORAL_POSITIONED)),
                new InstantCommand(
                        () -> RobotContainer.elevatorHead.set(CoralShooterStates.OFF), RobotContainer.elevatorHead));
    }

    public static boolean isRunning() {
        return running;
    }

    public static void setRunning(boolean running) {
        CoralIntakeSimple.running = running;
    }
}
