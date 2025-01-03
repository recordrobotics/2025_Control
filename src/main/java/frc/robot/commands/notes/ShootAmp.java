package frc.robot.commands.notes;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Crashbar;
import frc.robot.commands.KillSpecified;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import frc.robot.subsystems.Shooter.ShooterStates;

public class ShootAmp extends SequentialCommandGroup {

  private static Channel _channel;
  private static Shooter _shooter;
  private static Crashbar _crashbar;

  /** Number of seconds it takes for the flywheel to spin up */
  private final double flywheelSpinupTime = 0.3;
  private final double crashbarExtendTime = 0.4;
  /** Number of seconds it takes to shoot once the flywheel h as been spun up */
  private final double shootTime = 0.7;

  public ShootAmp (Channel channel, Shooter shooter, Crashbar crashbar) {
    _channel = channel;
    _shooter = shooter;
    _crashbar = crashbar;
    addRequirements(channel);
    addRequirements(shooter);
    addRequirements(crashbar);

    final Runnable killSpecified = () -> new KillSpecified(_shooter, _channel, _crashbar);

    addCommands(
      new InstantCommand(()->_shooter.toggle(ShooterStates.AMP), _shooter).handleInterrupt(killSpecified),
      new InstantCommand(()->_crashbar.toggle(CrashbarStates.EXTENDED), _crashbar).handleInterrupt(killSpecified),
      new WaitCommand(Math.max(flywheelSpinupTime, crashbarExtendTime)),
      new InstantCommand(()->_channel.toggle(ChannelStates.SHOOT), _channel).handleInterrupt(killSpecified),
      new WaitCommand(shootTime),
      new InstantCommand(()-> _shooter.toggle(ShooterStates.OFF), _shooter).handleInterrupt(killSpecified),
      new InstantCommand(()-> _channel.toggle(ChannelStates.OFF), _channel).handleInterrupt(killSpecified),
      new InstantCommand(()-> _crashbar.toggle(CrashbarStates.RETRACTED), _crashbar).handleInterrupt(killSpecified)
    );
  }
}

// TODO: use handle interrupt with a kill command