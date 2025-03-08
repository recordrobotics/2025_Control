package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.CoralShooterToReef;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralShoot extends SequentialCommandGroup {
  public CoralShoot() {
    addRequirements(RobotContainer.coralShooter);

    addCommands(
        new ScheduleCommand(
            RobotContainer.lights
                .coralShooter
                .runPattern(Constants.Lights.coralShooterPattern)
                .onlyWhile(this::isScheduled)),
        new ScheduleCommand(
            RobotContainer.lights
                .stateVisualizer
                .runPattern(Constants.Lights.coralScorePattern)
                .onlyWhile(this::isScheduled)),
        new InstantCommand(
            () -> {
              if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.L4
                  || RobotContainer.elevator.getNearestHeight() == ElevatorHeight.BARGE_ALAGAE)
                RobotContainer.coralShooter.toggle(CoralShooterStates.OUT_BACKWARD);
              else RobotContainer.coralShooter.toggle(CoralShooterStates.OUT_FORWARD);
            },
            RobotContainer.coralShooter),
        // Make sure coral left
        new CoralShooterToReef()
            .simulateFor(new WaitUntilCommand(() -> !RobotContainer.coralShooter.hasCoral())),
        new WaitCommand(Constants.CoralShooter.SHOOT_TIME),
        new InstantCommand(
            () -> RobotContainer.coralShooter.toggle(CoralShooterStates.OFF),
            RobotContainer.coralShooter),
        new ScheduleCommand(
            RobotContainer.lights
                .coralShooter
                .runPattern(Constants.Lights.FLASHING_GREEN)
                .alongWith(
                    RobotContainer.lights.stateVisualizer.runPattern(
                        Constants.Lights.PULSATING_GREEN))
                .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
  }
}
