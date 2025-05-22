package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;

public class ManualElevator extends Command {
  public ManualElevator() {
    addRequirements(RobotContainer.elevator);
  }

  private double height = 0;

  @Override
  public void initialize() {
    height = RobotContainer.elevator.getCurrentHeight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AbstractControl controls = DashboardUI.Overview.getControl();

    LinearVelocity manualElevatorVelocity = controls.getManualElevatorVelocity();
    height += manualElevatorVelocity.times(Milliseconds.of(20)).in(Meters);
    height =
        MathUtil.clamp(height, Constants.Elevator.STARTING_HEIGHT, Constants.Elevator.MAX_HEIGHT);

    // RobotContainer.elevator.set(height);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
