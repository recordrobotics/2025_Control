
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climbers extends SubsystemBase {

    private DoubleSolenoid solenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM, 
        RobotMap.Climbers.FORWARD_PORT,
        RobotMap.Climbers.REVERSE_PORT
    );

    public Climbers() {
        solenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void toggle(ClimberStates state) {
        switch (state) {
            case UP:
                solenoid.set(DoubleSolenoid.Value.kForward);
                break;
            case DOWN:
                solenoid.set(DoubleSolenoid.Value.kReverse);
                break;
            default:
                solenoid.set(DoubleSolenoid.Value.kOff);
                break;
        }
    }

    public enum ClimberStates {
        UP,
        DOWN,
        OFF;
    }
}