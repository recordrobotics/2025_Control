package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;

public class ManualShooter extends Command {

  private Shooter _shooter;
  private ShooterStates _state;

  public ManualShooter(Shooter shooter, ShooterStates state) {
    _shooter = shooter;
    _state = state;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shooter.toggle(_state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.toggle(ShooterStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
