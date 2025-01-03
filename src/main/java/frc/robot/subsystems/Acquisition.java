
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.ShuffleboardUI;

public class Acquisition extends SubsystemBase {
    private Spark acquisitionMotor = new Spark(RobotMap.Acquisition.ACQUISITION_MOTOR_ID);
    private static final double acquisitionDefaultSpeed = Constants.Acquisition.ACQUISITION_SPEED;
    public AcquisitionStates acquisitionState = AcquisitionStates.OFF;

    public Acquisition() {
        toggle(AcquisitionStates.OFF);
        var widget = ShuffleboardUI.Overview.getTab().addDouble("Acquisition", ()->acquisitionMotor.get());
        widget.withWidget(BuiltInWidgets.kNumberBar);
        widget.withPosition(6, 0);
        widget.withSize(2, 1);

        widget = ShuffleboardUI.Autonomous.getTab().addDouble("Acquisition", ()->acquisitionMotor.get());
        widget.withWidget(BuiltInWidgets.kNumberBar);
        widget.withPosition(9, 0);
        widget.withSize(2, 1);

        var widget1 = ShuffleboardUI.Test.getTab().add("Acquisition", acquisitionMotor);
        widget1.withWidget(BuiltInWidgets.kMotorController);
    }

    public void toggle(AcquisitionStates state, double speed) {
        acquisitionState = state;
        switch (state) {
            case IN:
                acquisitionMotor.set(speed);
                break;
            case REVERSE:
                acquisitionMotor.set(-speed);
                break;
            default:
                acquisitionMotor.set(0);
                break;
        }
    }

    public void toggle(AcquisitionStates state) {
        toggle(state, acquisitionDefaultSpeed);
    }

    public enum AcquisitionStates {
        IN,
        REVERSE,
        OFF;
    }
}
