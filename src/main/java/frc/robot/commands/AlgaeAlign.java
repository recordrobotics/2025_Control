package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Game.AlgaePosition;

public class AlgaeAlign {

  public static Command alignTarget(AlgaePosition pole, boolean repeatedly) {
    return GameAlign.alignTarget(
        () -> pole.getPose(), new Transform2d(-0.3, 0, Rotation2d.kZero), repeatedly);
  }
}
