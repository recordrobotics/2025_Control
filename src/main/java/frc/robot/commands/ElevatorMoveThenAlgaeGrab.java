package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.simulation.AlgaeGrabberSim;
import frc.robot.subsystems.ElevatorHead.AlgaeGrabberStates;

public class ElevatorMoveThenAlgaeGrab extends SequentialCommandGroup {

  private Command algaeGrabberLightsCommand;

  private ElevatorMoveThenAlgaeGrab(ElevatorHeight targetHeight) {
    algaeGrabberLightsCommand =
        RobotContainer.lights.algaeGrabber.runPattern(Constants.Lights.PULSATING_ORANGE);

    addRequirements(RobotContainer.elevatorHead);

    addCommands(
        new ScheduleCommand(
            RobotContainer.lights
                .elevator
                .runPattern(Constants.Lights.elevatorPattern)
                .onlyWhile(this::isScheduled)),
        new ElevatorMove(targetHeight).asProxy(),
        new ScheduleCommand(algaeGrabberLightsCommand),
        new InstantCommand(
            () -> {
              if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE)
                RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.INTAKE_GROUND);
              else RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.INTAKE_REEF);
            },
            RobotContainer.elevatorHead),
        new AlgaeGrabberSim(0.2)
            .simulateFor(new WaitUntilCommand(RobotContainer.elevatorHead::hasAlgae)),
        new InstantCommand(
            () -> {
              if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE)
                RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.HOLD_GROUND);
              else RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.HOLD_REEF);
            },
            RobotContainer.elevatorHead),
        new InstantCommand(algaeGrabberLightsCommand::cancel),
        new ScheduleCommand(
            RobotContainer.lights
                .algaeGrabber
                .runPattern(Constants.Lights.FLASHING_GREEN)
                .alongWith(
                    RobotContainer.lights.stateVisualizer.runPattern(
                        Constants.Lights.PULSATING_GREEN))
                .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
  }

  private void handleInterrupt() {
    algaeGrabberLightsCommand.cancel();
    RobotContainer.elevatorHead.toggle(AlgaeGrabberStates.OFF);
  }

  public static Command create(ElevatorHeight targetHeight) {
    var cmd = new ElevatorMoveThenAlgaeGrab(targetHeight);
    return cmd.handleInterrupt(cmd::handleInterrupt);
  }
}
