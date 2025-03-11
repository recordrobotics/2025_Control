package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralIntakeFromSourceSim;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralIntakeFromSource extends SequentialCommandGroup {
  public CoralIntakeFromSource() {
    addCommands(
        new ScheduleCommand(
            RobotContainer.lights
                .elevator
                .runPattern(Constants.Lights.elevatorPattern)
                .onlyWhile(this::isScheduled)),
        new ScheduleCommand(
            RobotContainer.lights
                .coralIntake
                .runPattern(Constants.Lights.coralIntakePattern)
                .onlyWhile(this::isScheduled)),
        new ScheduleCommand(
            RobotContainer.lights
                .coralShooter
                .runPattern(Constants.Lights.coralShooterPattern)
                .onlyWhile(this::isScheduled)),
        new ScheduleCommand(
            RobotContainer.lights
                .stateVisualizer
                .runPattern(Constants.Lights.PULSATING_ORANGE)
                .onlyWhile(this::isScheduled)),
        new InstantCommand(
            () -> {
              RobotContainer.coralIntake.toggleArm(IntakeArmStates.INTAKE);
              RobotContainer.coralIntake.toggle(CoralIntakeStates.REVERSE);
            },
            RobotContainer.coralIntake),
        // start moving elevator to intake position
        new ElevatorMove(ElevatorHeight.INTAKE).asProxy(),
        new InstantCommand(
            () -> RobotContainer.coralShooter.toggle(CoralShooterStates.INTAKE),
            RobotContainer.coralShooter),
        new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
        new CoralIntakeFromSourceSim()
            .simulateFor(new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral())),
        new InstantCommand(
            () -> {
              RobotContainer.coralIntake.toggleArm(IntakeArmStates.UP);
              RobotContainer.coralIntake.toggle(CoralIntakeStates.OFF);
            },
            RobotContainer.coralIntake),
        // move coral a set distance
        new InstantCommand(
            () -> RobotContainer.coralShooter.moveBy(Constants.CoralShooter.CORAL_INTAKE_DISTANCE),
            RobotContainer.coralShooter),
        new WaitUntilCommand(() -> RobotContainer.coralShooter.positionAtGoal()),
        new InstantCommand(
            () -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF),
            RobotContainer.coralShooter),
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
