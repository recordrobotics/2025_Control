// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.shuffleboard.ShuffleboardUI;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends KillableSubsystem {

    private CANSparkMax flywheelL = new CANSparkMax(RobotMap.Shooter.FLYWHEEL_MOTOR_LEFT_DEVICE_ID, MotorType.kBrushless);
    private CANSparkMax flywheelR = new CANSparkMax(RobotMap.Shooter.FLYWHEEL_MOTOR_RIGHT_DEVICE_ID, MotorType.kBrushless);

    public Shooter() {
        toggle(ShooterStates.OFF);
        ShuffleboardUI.Test.addSlider("Flywheel Left", flywheelL.get(), -1, 1).subscribe(flywheelL::set);
        ShuffleboardUI.Test.addSlider("Flywheel Right", flywheelR.get(), -1, 1).subscribe(flywheelR::set);
    }

    public enum ShooterStates {
        SPEAKER,
        AMP,
        REVERSE,
        OFF;
    }

    public void toggle(double speedL, double speedR) {
        flywheelL.set(-speedL);
        flywheelR.set(speedR);
    }

    public void toggle(double speed) {
        toggle(speed, speed);
    }

    public void toggle(ShooterStates state) {
        switch (state) {
            case SPEAKER:
                toggle(Constants.Shooter.SPEAKER_SPEED);
                break;
            case AMP:
                toggle(Constants.Shooter.AMP_SPEED);
                break;
            case REVERSE:
                toggle(Constants.Shooter.REVERSE_SPEED);
                break;
            default:
                toggle(0);
                break;
        }
    }

    @Override
    public void kill() {
        toggle(ShooterStates.OFF);
    }
}
