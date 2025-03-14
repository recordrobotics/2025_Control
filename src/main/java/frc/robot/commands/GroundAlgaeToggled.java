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
    ElevatorMoveThenAlgaeGrab.create(targetHeight, false)
        .andThen(new ElevatorMoveThenAlgaeGrabEnd(targetHeight, true))
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
              RobotContainer.elevatorHead.hasAlgae() ? targetHeight : ElevatorHeight.BOTTOM, true)
          .handleInterrupt(this::cancel)
          .schedule();
    }
  }
}
