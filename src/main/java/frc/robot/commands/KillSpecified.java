package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Channel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Crashbar;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Channel.ChannelStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.subsystems.Acquisition.AcquisitionStates;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import frc.robot.subsystems.Climbers.ClimberStates;;


public class KillSpecified extends Command {

    private SubsystemBase[] _subsytems;
    private Boolean _shouldContinuouslyExecute;

    /**
     * Kills whatever subsystem you put into the system
     * @param subsystems
     * 
     */
    public KillSpecified (SubsystemBase... subsystems) {
        addRequirements(subsystems); //TODO: is this necessary/helpful?
        _subsytems = subsystems;
        _shouldContinuouslyExecute = false;
    }

    /**
     * 
     * @param execute whether or not the command should run continuously
     * @param subsystems 
     */
    public KillSpecified (Boolean shouldContinuouslyExecute, SubsystemBase... subsystems) {
        addRequirements(subsystems); //TODO: is this necessary/helpful?
        _subsytems = subsystems;
        _shouldContinuouslyExecute = shouldContinuouslyExecute;
    }

    @Override
    public void initialize() {
        for (SubsystemBase subsystem: _subsytems) {

            if (subsystem instanceof Channel) {
                ((Channel)subsystem).toggle(ChannelStates.OFF);
            }

            if (subsystem instanceof Shooter) {
                ((Shooter)subsystem).toggle(ShooterStates.OFF);
            }

            if (subsystem instanceof Acquisition) {
                ((Acquisition)subsystem).toggle(AcquisitionStates.OFF);
            }

            if (subsystem instanceof Crashbar) {
                ((Crashbar)subsystem).toggle(CrashbarStates.RETRACTED);
            }

            if (subsystem instanceof Climbers) {
                ((Climbers)subsystem).toggle(ClimberStates.OFF);
            }

            if (subsystem instanceof Drivetrain) {
                ((Drivetrain)subsystem).stop();
            }
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return !_shouldContinuouslyExecute;
    }

}
