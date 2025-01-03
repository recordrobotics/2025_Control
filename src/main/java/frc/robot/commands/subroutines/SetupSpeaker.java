package frc.robot.commands.subroutines;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterStates;


public class SetupSpeaker extends SequentialCommandGroup {

  private static Shooter _shooter;

  /**
   * Sets up the shooter to get the 
   * @param shooter
   */
  public SetupSpeaker (Shooter shooter) {
    _shooter = shooter;

    final Runnable killSpecified = () -> new KillSpecified(_shooter);

    addCommands(
      new InstantCommand(()->_shooter.toggle(ShooterStates.SPEAKER), _shooter).handleInterrupt(killSpecified)
    );
  }
}