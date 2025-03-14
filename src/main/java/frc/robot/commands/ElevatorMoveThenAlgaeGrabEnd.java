package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabberStates;

public class ElevatorMoveThenAlgaeGrabEnd extends SequentialCommandGroup {
  public ElevatorMoveThenAlgaeGrabEnd(ElevatorHeight targetHeight, boolean withProxy) {
    addRequirements(RobotContainer.algaeGrabber);

    addCommands(
        Commands.either(
            Commands.either(
                new InstantCommand(
                    () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.HOLD_GROUND),
                    RobotContainer.algaeGrabber),
                new InstantCommand(
                    () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.HOLD_REEF),
                    RobotContainer.algaeGrabber),
                () -> targetHeight == ElevatorHeight.GROUND_ALGAE),
            new InstantCommand(
                () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF),
                RobotContainer.algaeGrabber),
            RobotContainer.algaeGrabber::hasAlgae),
        withProxy
            ? Commands.either(
                    new ElevatorMove(ElevatorHeight.GROUND_ALGAE),
                    new ElevatorMove(ElevatorHeight.BOTTOM),
                    () -> targetHeight == ElevatorHeight.GROUND_ALGAE)
                .asProxy()
            : Commands.either(
                new ElevatorMove(ElevatorHeight.GROUND_ALGAE),
                new ElevatorMove(ElevatorHeight.BOTTOM),
                () -> targetHeight == ElevatorHeight.GROUND_ALGAE),
        new ScheduleCommand(
            RobotContainer.lights
                .elevator
                .runPattern(Constants.Lights.FLASHING_GREEN)
                .alongWith(
                    RobotContainer.lights.stateVisualizer.runPattern(
                        Constants.Lights.PULSATING_GREEN))
                .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
  }
}
