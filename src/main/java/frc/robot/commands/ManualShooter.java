package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.IControlInput;
import frc.robot.subsystems.Shooter;

public class ManualShooter extends Command{

    private Shooter _shooter;
    private IControlInput _controls;

     public ManualShooter(Shooter shooter, IControlInput controls) {
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
    if(_controls.spinFlywheel()) _shooter.shoot(3);
    else if(_controls.getAcquisition()) _shooter.shoot(2);
    else _shooter.shoot(1);
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
