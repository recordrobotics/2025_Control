package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterStates;

public class ManualShooter extends Command {

  private ShooterStates _state;

  public ManualShooter(ShooterStates state) {
    _state = state;
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.toggle(_state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.toggle(ShooterStates.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
