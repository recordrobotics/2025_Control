package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeToElevator;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;

public class CoralIntakeFromGroundUp extends SequentialCommandGroup {
  public CoralIntakeFromGroundUp() {
    addRequirements(RobotContainer.coralIntake);

    addRequirements(RobotContainer.elevatorHead);
    addRequirements(RobotContainer.elevator);

    addCommands(
        // raise the arm
        new InstantCommand(
            () -> RobotContainer.coralIntake.set(CoralIntakeState.PUSH_READY),
            RobotContainer.coralIntake),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
        new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
        new ScheduleCommand(
            RobotContainer.lights
                .coralIntake
                .runPattern(Constants.Lights.PULSATING_ORANGE)
                .onlyWhile(this::isScheduled)),
        // once both the arm and elevator are at goal, start elevator intake
        new InstantCommand(
                () -> RobotContainer.elevatorHead.set(CoralShooterStates.INTAKE),
                RobotContainer.elevatorHead)
            .andThen(
                // push coral out
                new InstantCommand(
                    () -> RobotContainer.coralIntake.set(CoralIntakeState.PUSH_OUT),
                    RobotContainer.coralIntake))
            .andThen(
                // wait for elevator to have coral
                new CoralIntakeToElevator()
                    .simulateFor(
                        new WaitUntilCommand(() -> RobotContainer.elevatorHead.hasCoral())))
            .andThen(
                new InstantCommand(
                    () -> {
                      RobotContainer.coralIntake.set(CoralIntakeState.UP);
                    },
                    RobotContainer.coralIntake))
            .andThen(
                // move coral a set distance
                new InstantCommand(
                    () ->
                        RobotContainer.elevatorHead.moveBy(
                            Constants.ElevatorHead.CORAL_INTAKE_DISTANCE),
                    RobotContainer.elevatorHead))
            .andThen(
                new WaitUntilCommand(() -> RobotContainer.elevatorHead.positionAtGoal()),
                // stop elevator intake
                new InstantCommand(
                    () -> RobotContainer.elevatorHead.set(CoralShooterStates.OFF),
                    RobotContainer.elevatorHead)),
        new ScheduleCommand(
            RobotContainer.lights
                .elevator
                .runPattern(Constants.Lights.FLASHING_GREEN)
                .alongWith(
                    RobotContainer.lights.coralIntake.runPattern(Constants.Lights.FLASHING_GREEN))
                .alongWith(
                    RobotContainer.lights.coralShooter.runPattern(Constants.Lights.FLASHING_GREEN))
                .alongWith(
                    RobotContainer.lights.stateVisualizer.runPattern(
                        Constants.Lights.PULSATING_GREEN))
                .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
  }
}
