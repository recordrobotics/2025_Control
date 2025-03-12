package frc.robot.commands.hybrid;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.Align;
import frc.robot.commands.CoralShoot;
import frc.robot.commands.ElevatorMove;
import frc.robot.control.AbstractControl.AutoScoreDirection;

public class AutoScore extends SequentialCommandGroup {

  public AutoScore(AutoScoreDirection direction) {
    addRequirements(RobotContainer.drivetrain);

    addCommands(
        Align.createForReef(direction, 0.01, 0.05).withTimeout(1.0),
        new CoralShoot(),
        Align.createForReefBackaway(direction, 0.01, 0.05)
            .withTimeout(1.0)
            .andThen(new ElevatorMove(ElevatorHeight.BOTTOM)));
  }
}
