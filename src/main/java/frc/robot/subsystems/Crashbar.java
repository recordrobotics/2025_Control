package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotMap;

public class Crashbar extends KillableSubsystem {

    // Creates solenoid
    private DoubleSolenoid solenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        RobotMap.Crashbar.FORWARD_PORT,
        RobotMap.Crashbar.REVERSE_PORT
    );

    public Crashbar() {
        toggle(CrashbarStates.OFF);
    }

    public enum CrashbarStates {
        EXTENDED,
        RETRACTED,
        OFF;
    }

    public void toggle(CrashbarStates state) {
        switch (state) {
            case EXTENDED:
                solenoid.set(DoubleSolenoid.Value.kReverse);
                break;
            case RETRACTED:
                solenoid.set(DoubleSolenoid.Value.kForward);
                break;
            default:
                solenoid.set(DoubleSolenoid.Value.kOff);
                break;
        }
    }

    @Override
    public void kill() {
        toggle(CrashbarStates.OFF);
    }

    
}