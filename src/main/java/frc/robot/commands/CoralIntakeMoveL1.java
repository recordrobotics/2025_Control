package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeArmStates;

public class CoralIntakeMoveL1 extends SequentialCommandGroup {

  public CoralIntakeMoveL1() {
    addCommands(
        new ScheduleCommand(
            RobotContainer.lights
                .coralIntake
                .runPattern(Constants.Lights.coralIntakePattern)
                .onlyWhile(this::isScheduled)),
        new ScheduleCommand(
            RobotContainer.lights
                .stateVisualizer
                .runPattern(Constants.Lights.coralScorePattern)
                .onlyWhile(this::isScheduled)),
        new InstantCommand(
            () -> {
              RobotContainer.coralIntake.toggleArm(IntakeArmStates.SCORE_L1);
              RobotContainer.coralIntake.toggle(CoralIntakeStates.INTAKE);
            },
            RobotContainer.coralIntake),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()));
  }
}
