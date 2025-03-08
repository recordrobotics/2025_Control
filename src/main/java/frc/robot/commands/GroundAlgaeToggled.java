package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;

public class GroundAlgaeToggled extends Command {
  private ElevatorHeight targetHeight;

  private boolean isRunning = false;

  public GroundAlgaeToggled(ElevatorHeight targetHeight) {
    this.targetHeight = targetHeight;
    addRequirements(RobotContainer.elevatorMoveToggleRequirement);
  }

  @Override
  public void initialize() {
    isRunning = true;
    ElevatorMoveThenAlgaeGrab.create(targetHeight)
        .andThen(new ElevatorMoveThenAlgaeGrabEnd(targetHeight))
        .andThen(() -> isRunning = false)
        .handleInterrupt(this::cancel)
        .schedule();
  }

  @Override
  public boolean isFinished() {
    return !isRunning;
  }

  @Override
  public void end(boolean interrupted) {
    isRunning = false;
    if (interrupted) {
      new ElevatorMoveThenAlgaeGrabEnd(
              RobotContainer.algaeGrabber.hasAlgae() ? targetHeight : ElevatorHeight.BOTTOM)
          .handleInterrupt(this::cancel)
          .schedule();
    }
  }
}
