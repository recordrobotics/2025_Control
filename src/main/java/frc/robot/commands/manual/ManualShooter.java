package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.Shooter;

public class ManualShooter extends Command {

  private Shooter _shooter;
  private DoubleControl _controls;

  public ManualShooter(Shooter shooter, DoubleControl controls) {
    _shooter = shooter;
    _controls = controls;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //
    if (_controls.getShoot())
      _shooter.shoot();
    else
      _shooter.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
