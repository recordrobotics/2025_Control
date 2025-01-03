// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.ShuffleboardUI;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {

    private CANSparkMax flywheelL = new CANSparkMax(RobotMap.Shooter.FLYWHEEL_MOTOR_LEFT_DEVICE_ID, MotorType.kBrushless);
    private CANSparkMax flywheelR = new CANSparkMax(RobotMap.Shooter.FLYWHEEL_MOTOR_RIGHT_DEVICE_ID, MotorType.kBrushless);

    GenericEntry widgetL;
    GenericEntry widgetR;

    public Shooter() {
        toggle(ShooterStates.OFF);

        widgetL = ShuffleboardUI.Test.getTab().add("Flywheel Left", flywheelL.get())
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();

        widgetR = ShuffleboardUI.Test.getTab().add("Flywheel Right", flywheelR.get())
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();
    }

    public void testPeriodic(){
        flywheelL.set(widgetL.getDouble(0));
        flywheelR.set(widgetR.getDouble(0));
    }

    public void toggle(double speedL, double speedR) {
        flywheelL.set(-speedL);
        flywheelR.set(speedR);
    }

    public void toggle(double speed) {
        flywheelL.set(-speed);
        flywheelR.set(speed);
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

    public enum ShooterStates {
        SPEAKER,
        AMP,
        REVERSE,
        OFF;
    }
}
