package frc.robot.commands.manual;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.Climbers;

public class ManualClimbers extends Command {

  private Climbers _climbers;
  private DoubleControl _controls;

  public ManualClimbers(Climbers climbers, DoubleControl controls) {
    _climbers = climbers;
    _controls = controls;
    addRequirements(climbers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("chainUp", _controls.getChainUp());
    if (_controls.getChainUp()) {
      _climbers.chainUp();
    } else if (_controls.getChainDown()) {
      _climbers.chainDown();
    } else {
      _climbers.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _climbers.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
