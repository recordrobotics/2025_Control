package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Lights.LightSegments;
import frc.robot.RobotContainer;

public class SuccessfulCompletion extends SequentialCommandGroup {
  public SuccessfulCompletion(
      boolean elevator, boolean groundAlgae, boolean coralIntake, boolean coralShooter) {
    if (elevator) {
      addFlashGreenCommands(LightSegments.ELEVATOR);
    }
    if (groundAlgae) {
      addFlashGreenCommands(LightSegments.GROUND_ALGAE);
    }
    if (coralIntake) {
      addFlashGreenCommands(LightSegments.CORAL_INTAKE);
    }
    if (coralShooter) {
      addFlashGreenCommands(LightSegments.CORAL_SHOOTER);
    }
  }

  private void addFlashGreenCommands(LightSegments segment) {
    addCommands(
        new InstantCommand(
            () ->
                new SequentialCommandGroup(
                        new InstantCommand(
                            () ->
                                RobotContainer.lights.patterns.put(
                                    segment, () -> Constants.Lights.PULSATING_GREEN)),
                        new WaitCommand(1),
                        new InstantCommand(
                            () ->
                                RobotContainer.lights.patterns.put(
                                    segment, () -> Constants.Lights.OFF)))
                    .schedule()));
  }
}
