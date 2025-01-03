package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.IControlInput;
import frc.robot.subsystems.Climbers;

public class ManualClimbers extends Command{

    private Climbers _climbers;
    private IControlInput _controls;

     public ManualClimbers(Climbers climbers, IControlInput controls) {
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
    if(_controls.toggleClimbers()) _climbers.toggle();
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
