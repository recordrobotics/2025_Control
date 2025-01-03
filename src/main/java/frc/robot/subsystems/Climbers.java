
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climbers extends SubsystemBase {

    private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.Climbers.LEFT_FORWARD_PORT,
            RobotMap.Climbers.LEFT_REVERSE_PORT);

    public Climbers() {
        solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void chainUp() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void chainDown() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Turns off solenoid, does not reverse it
     */
    public void stop() {
        solenoid.set(DoubleSolenoid.Value.kOff);
    }
}
