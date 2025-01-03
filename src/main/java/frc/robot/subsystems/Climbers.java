
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climbers extends SubsystemBase{

    private Solenoid left = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.Climbers.left);
    private Solenoid right = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.Climbers.right);

    public Climbers() {
        left.close();
        right.close();
    }

    public void toggle(){
        left.toggle();
        right.toggle();
    }
    
}
