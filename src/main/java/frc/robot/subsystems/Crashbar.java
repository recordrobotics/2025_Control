package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Crashbar extends SubsystemBase {
    private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            RobotMap.Crashbar.FORWARD_PORT,
            RobotMap.Crashbar.REVERSE_PORT);

    public Crashbar() {
        solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void extend() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop() {
        solenoid.set(DoubleSolenoid.Value.kOff);
    }
}
