package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;

public class PlannedAuto extends SequentialCommandGroup {
  public PlannedAuto() {
    addCommands(
        DashboardUI.Autonomous.getAutoChooser()
            .finallyDo(
                () -> {
                  DriverStation.reportError("AUTO ENDED!!!!!!!!!!!", false);
                }),
        new InstantCommand(() -> RobotContainer.drivetrain.kill()));
  }
}
