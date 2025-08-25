package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
import frc.robot.subsystems.ElevatorHead.GamePiece;

public class CoralShoot extends SequentialCommandGroup {

    private static boolean failedToShoot = false;

    public CoralShoot() {
        addRequirements(RobotContainer.elevatorHead);

        addCommands(
                new InstantCommand(() -> setFailedToShoot(false)),
                new ScheduleCommand(RobotContainer.lights
                        .coralShooter
                        .runPattern(Constants.Lights.elevatorHeadPattern)
                        .onlyWhile(this::isScheduled)),
                new ScheduleCommand(RobotContainer.lights
                        .stateVisualizer
                        .runPattern(Constants.Lights.coralScorePattern)
                        .onlyWhile(this::isScheduled)),
                new InstantCommand(
                        () -> {
                            if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.L4
                                    || RobotContainer.elevator.getNearestHeight() == ElevatorHeight.BARGE_ALGAE)
                                RobotContainer.elevatorHead.set(CoralShooterStates.OUT_BACKWARD);
                            else RobotContainer.elevatorHead.set(CoralShooterStates.OUT_FORWARD);
                        },
                        RobotContainer.elevatorHead),
                // Make sure coral left
                new WaitUntilCommand(() ->
                                !RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.CORAL))
                        .raceWith(new WaitCommand(Constants.ElevatorHead.SHOOT_STALL_TIME)
                                .andThen(new WaitUntilCommand(() -> {
                                    if (RobotContainer.elevator.getNearestHeight() != ElevatorHeight.L4) return false;

                                    boolean stalled = Math.abs(RobotContainer.elevatorHead.getVelocity())
                                            < Constants.ElevatorHead.SHOOT_STALL_THRESHOLD;

                                    if (stalled) {
                                        setFailedToShoot(true);
                                    }

                                    return stalled;
                                }))),
                Commands.either(
                        new InstantCommand(
                                        () -> RobotContainer.elevatorHead.set(CoralShooterStates.INTAKE),
                                        RobotContainer.elevatorHead)
                                .andThen(
                                        // wait for elevator to have coral
                                        new WaitUntilCommand(() -> RobotContainer.elevatorHead
                                                .getGamePiece()
                                                .atLeast(GamePiece.CORAL)))
                                .andThen(
                                        // move coral a set distance
                                        new InstantCommand(
                                                () -> RobotContainer.elevatorHead.moveBy(
                                                        Constants.ElevatorHead.CORAL_INTAKE_DISTANCE),
                                                RobotContainer.elevatorHead))
                                .andThen(new WaitUntilCommand(() -> RobotContainer.elevatorHead
                                        .getGamePiece()
                                        .atLeast(GamePiece.CORAL_POSITIONED))),
                        new WaitCommand(Constants.ElevatorHead.SHOOT_TIME),
                        () -> failedToShoot),
                new InstantCommand(
                        () -> RobotContainer.elevatorHead.set(CoralShooterStates.OFF), RobotContainer.elevatorHead),
                new ScheduleCommand(RobotContainer.lights
                        .coralShooter
                        .runPattern(Constants.Lights.FLASHING_GREEN)
                        .alongWith(RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.PULSATING_GREEN))
                        .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
    }

    private static void setFailedToShoot(boolean value) {
        failedToShoot = value;
    }

    public static boolean failedToShoot() {
        return failedToShoot;
    }
}
