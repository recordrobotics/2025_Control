package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.AlgaeGrabberToProcessor;
import frc.robot.subsystems.AlgaeGrabber.AlgaeGrabberStates;

public class ProcessorScore extends SequentialCommandGroup {

  public ProcessorScore(boolean withProxy) {
    addRequirements(RobotContainer.algaeGrabber);

    addCommands(
        new ScheduleCommand(
            RobotContainer.lights
                .stateVisualizer
                .runPattern(Constants.Lights.algaeScorePattern)
                .onlyWhile(this::isScheduled)),
        new ScheduleCommand(
            RobotContainer.lights
                .algaeGrabber
                .runPattern(Constants.Lights.PULSATING_ORANGE)
                .onlyWhile(this::isScheduled)),
        Commands.either(
            maybieProxy(withProxy, new ElevatorMove(ElevatorHeight.GROUND_ALGAE_PROCESSOR))
                .andThen(
                    new InstantCommand(
                        () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OUT_GROUND),
                        RobotContainer.algaeGrabber))
                .andThen(
                    new AlgaeGrabberToProcessor()
                        .simulateFor(new WaitCommand(Constants.AlgaeGrabber.SHOOT_TIME_GROUND)))
                .andThen(
                    new InstantCommand(
                        () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF),
                        RobotContainer.algaeGrabber))
                .andThen(maybieProxy(withProxy, new ElevatorMove(ElevatorHeight.BOTTOM))),
            Commands.either(
                new InstantCommand(
                        () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.SHOOT_BARGE),
                        RobotContainer.algaeGrabber)
                    .andThen(
                        new AlgaeGrabberToProcessor()
                            .simulateFor(new WaitCommand(Constants.AlgaeGrabber.SHOOT_TIME_BARGE)))
                    .andThen(
                        new InstantCommand(
                            () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF),
                            RobotContainer.algaeGrabber)),
                maybieProxy(withProxy, new ElevatorMove(ElevatorHeight.PROCESSOR_SCORE))
                    .andThen(
                        new InstantCommand(
                            () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OUT_REEF),
                            RobotContainer.algaeGrabber))
                    .andThen(
                        new AlgaeGrabberToProcessor()
                            .simulateFor(new WaitCommand(Constants.AlgaeGrabber.SHOOT_TIME_REEF)))
                    .andThen(
                        new InstantCommand(
                            () -> RobotContainer.algaeGrabber.toggle(AlgaeGrabberStates.OFF),
                            RobotContainer.algaeGrabber))
                    .andThen(maybieProxy(withProxy, new ElevatorMove(ElevatorHeight.BOTTOM))),
                () -> RobotContainer.elevator.getNearestHeight() == ElevatorHeight.BARGE_ALAGAE),
            () -> RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE),
        new ScheduleCommand(
            RobotContainer.lights
                .algaeGrabber
                .runPattern(Constants.Lights.FLASHING_GREEN)
                .alongWith(
                    RobotContainer.lights.stateVisualizer.runPattern(
                        Constants.Lights.PULSATING_GREEN))
                .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
  }

  private Command maybieProxy(boolean withProxy, Command cmd) {
    return withProxy ? cmd.asProxy() : cmd;
  }
}
