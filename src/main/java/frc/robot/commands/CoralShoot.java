package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;

public class CoralShoot extends SequentialCommandGroup {
  public CoralShoot() {
    addRequirements(RobotContainer.elevatorHead);

    addCommands(
        new ScheduleCommand(
            RobotContainer.lights
                .coralShooter
                .runPattern(Constants.Lights.elevatorHeadPattern)
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
                RobotContainer.elevatorHead.set(CoralShooterStates.OUT_BACKWARD);
              else RobotContainer.elevatorHead.set(CoralShooterStates.OUT_FORWARD);
            },
            RobotContainer.elevatorHead),
        // Make sure coral left
        new WaitUntilCommand(() -> !RobotContainer.elevatorHead.hasCoral()),
        new WaitCommand(Constants.ElevatorHead.SHOOT_TIME),
        new InstantCommand(
            () -> RobotContainer.elevatorHead.set(CoralShooterStates.OFF),
            RobotContainer.elevatorHead),
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
