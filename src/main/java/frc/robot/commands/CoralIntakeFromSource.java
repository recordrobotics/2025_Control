package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;

public class CoralIntakeFromSource extends SequentialCommandGroup {
  public CoralIntakeFromSource() {
    addRequirements(RobotContainer.coralIntake);

    addCommands(
        new InstantCommand(() -> RobotContainer.elevator.moveTo(ElevatorHeight.INTAKE)),
        new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.ACQUIRE)),
        new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral()),
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.OFF)));
  }
}