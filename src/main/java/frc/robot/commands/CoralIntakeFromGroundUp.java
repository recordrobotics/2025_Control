package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeToElevator;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralIntakeFromGroundUp extends SequentialCommandGroup {
  public CoralIntakeFromGroundUp(boolean moveToElevator) {
    addRequirements(RobotContainer.coralIntake);

    if (moveToElevator) {
      addRequirements(RobotContainer.coralShooter);
      addRequirements(RobotContainer.elevator);
    }

    addCommands(
        // raise the arm
        new InstantCommand(
            () ->
                RobotContainer.coralIntake.toggleArm(
                    moveToElevator ? IntakeArmStates.INTAKE : IntakeArmStates.UP),
            RobotContainer.coralIntake),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
        new WaitUntilCommand(() -> !moveToElevator || RobotContainer.elevator.atGoal()),
        new ScheduleCommand(
            RobotContainer.lights
                .coralIntake
                .runPattern(Constants.Lights.PULSATING_ORANGE)
                .onlyWhile(this::isScheduled)),
        Commands.either(
            // once both the arm and elevator are at goal, start elevator intake
            new InstantCommand(
                    () -> RobotContainer.coralShooter.toggle(CoralShooterStates.INTAKE),
                    RobotContainer.coralShooter)
                .andThen(
                    // push coral out
                    new InstantCommand(
                        () -> RobotContainer.coralIntake.toggle(CoralIntakeStates.REVERSE),
                        RobotContainer.coralIntake))
                .andThen(
                    // wait for elevator to have coral
                    new CoralIntakeToElevator()
                        .simulateFor(
                            new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral())))
                .andThen(
                    new InstantCommand(
                        () -> {
                          RobotContainer.coralIntake.toggleArm(IntakeArmStates.UP);
                          RobotContainer.coralIntake.toggle(CoralIntakeStates.OFF);
                        },
                        RobotContainer.coralIntake))
                .andThen(
                    // move coral a set distance
                    new InstantCommand(
                        () ->
                            RobotContainer.coralShooter.moveBy(
                                Constants.CoralShooter.CORAL_INTAKE_DISTANCE),
                        RobotContainer.coralShooter))
                .andThen(
                    new WaitUntilCommand(() -> RobotContainer.coralShooter.positionAtGoal()),
                    // stop elevator intake
                    new InstantCommand(
                        () -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF),
                        RobotContainer.coralShooter)),
            new InstantCommand(
                () -> {
                  RobotContainer.coralIntake.toggle(CoralIntakeStates.INTAKE);
                },
                RobotContainer.coralIntake),
            () -> moveToElevator),
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
