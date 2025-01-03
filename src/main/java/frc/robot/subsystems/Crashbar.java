package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Crashbar extends SubsystemBase {

    // Creates solenoid
    private DoubleSolenoid solenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        RobotMap.Crashbar.FORWARD_PORT,
        RobotMap.Crashbar.REVERSE_PORT
    );

    public Crashbar() {
        solenoid.set(DoubleSolenoid.Value.kOff);
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

    public enum CrashbarStates {
        EXTENDED,
        RETRACTED,
        OFF;
    }
}