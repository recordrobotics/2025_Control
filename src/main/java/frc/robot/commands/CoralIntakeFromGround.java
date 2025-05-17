package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;

public class CoralIntakeFromGround extends SequentialCommandGroup {
  public CoralIntakeFromGround() {
    addRequirements(RobotContainer.coralIntake);
    addRequirements(RobotContainer.elevatorHead);

    addCommands(
        new ScheduleCommand(
            RobotContainer.lights
                .stateVisualizer
                .runPattern(Constants.Lights.PULSATING_ORANGE)
                .onlyWhile(this::isScheduled)),
        new ScheduleCommand(
            RobotContainer.lights
                .coralIntake
                .runPattern(Constants.Lights.coralIntakePattern)
                .onlyWhile(this::isScheduled)),
        new InstantCommand(
            () -> RobotContainer.coralIntake.set(CoralIntakeState.GROUND),
            RobotContainer.coralIntake),
        // start moving elevator to intake position
        new ScheduleCommand(new ElevatorMove(ElevatorHeight.INTAKE)),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
        new ScheduleCommand(
            RobotContainer.lights
                .coralIntake
                .runPattern(Constants.Lights.PULSATING_GREEN)
                .onlyWhile(this::isScheduled)));
  }
}
