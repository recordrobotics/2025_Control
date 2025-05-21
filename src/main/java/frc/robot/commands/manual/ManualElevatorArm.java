package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;

public class ManualElevatorArm extends Command {
  public ManualElevatorArm() {
    addRequirements(RobotContainer.elevatorArm);
  }

  private double angle = 0;

  @Override
  public void initialize() {
    angle = RobotContainer.elevatorArm.getArmAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AbstractControl controls = DashboardUI.Overview.getControl();

    AngularVelocity manualElevatorArmVelocity = controls.getManualElevatorArmVelocity();
    angle += manualElevatorArmVelocity.times(Milliseconds.of(20)).in(Radians);
    angle = MathUtil.clamp(angle, Constants.ElevatorArm.MIN_POS, Constants.ElevatorArm.MAX_POS);

    // RobotContainer.elevatorArm.set(angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
