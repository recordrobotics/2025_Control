package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeStates;
import frc.robot.subsystems.CoralIntake.IntakeServoStates;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;

public class CoralIntakeFromGround extends SequentialCommandGroup {
  public CoralIntakeFromGround() {
    addRequirements(RobotContainer.coralIntake);

    addCommands(
        new InstantCommand(() -> RobotContainer.elevator.moveTo(ElevatorHeight.INTAKE)),
        new InstantCommand(() -> RobotContainer.coralIntake.toggleServo(IntakeServoStates.DOWN)),
        new WaitUntilCommand(() -> RobotContainer.coralIntake.servoAtGoal()),
        new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
        new InstantCommand(() -> RobotContainer.coralShooter.toggle(CoralShooterStates.INTAKE)),
        new WaitUntilCommand(() -> RobotContainer.coralShooter.hasCoral()),
        new InstantCommand(() -> RobotContainer.coralIntake.toggle(CoralIntakeStates.OFF)));
  }
}
