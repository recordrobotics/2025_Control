// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

    private CANSparkMax flywheelLeft = new CANSparkMax(RobotMap.Shooter.FLYWHEEL_MOTOR_LEFT_DEVICE_ID,
            MotorType.kBrushless);
    private CANSparkMax flywheelRight = new CANSparkMax(RobotMap.Shooter.FLYWHEEL_MOTOR_RIGHT_DEVICE_ID,
            MotorType.kBrushless);

    public Shooter() {
        flywheelLeft.set(0);
        flywheelRight.set(0);

        SmartDashboard.putNumber("Shooter speed", 0.1);
    }

    public void shoot() {
        // get the flywheel speed from shuffleboard...
        double speed = SmartDashboard.getNumber("Shooter speed", 0.1);

        // ... clamp for safety ...
        speed = Math.max(0, Math.min(1, speed));

        // ... and try to not hit anyone!
        flywheelLeft.set(speed);
        flywheelRight.set(-speed);
    }

    public void stop() {
        flywheelLeft.set(0);
        flywheelRight.set(0);
    }
}
