package frc.robot.commands.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.RobotCoral;
import java.util.HashSet;

public class CoralIntakeToElevator extends SequentialCommandGroup implements SimulationCommand {

  private RobotCoral coral;

  public CoralIntakeToElevator() {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        new InstantCommand(() -> coral = RobotContainer.model.getRobotCoral()),
        // move coral to elevator
        new DeferredCommand(
            () ->
                (coral.poseSupplier.get() == null
                    ? Commands.none()
                    : new PoseAnimator(
                            coral
                                .poseSupplier
                                .get()
                                .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                            () ->
                                RobotContainer.model
                                    .elevator
                                    .getCoralIntakeEjectPose()
                                    .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                            p ->
                                coral.poseSupplier =
                                    () ->
                                        new Pose3d(RobotContainer.model.getRobot())
                                            .plus(
                                                new Transform3d(
                                                    p.getTranslation(), p.getRotation())),
                            0.1)
                        .andThen(
                            new PoseAnimator(
                                RobotContainer.model
                                    .elevator
                                    .getCoralIntakeEjectPose()
                                    .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                                () ->
                                    RobotContainer.model
                                        .elevator
                                        .getCoralIntakeEjectFinalPose()
                                        .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                                p ->
                                    coral.poseSupplier =
                                        () ->
                                            new Pose3d(RobotContainer.model.getRobot())
                                                .plus(
                                                    new Transform3d(
                                                        p.getTranslation(), p.getRotation())),
                                0.1))
                        .andThen(
                            new PoseAnimator(
                                RobotContainer.model
                                    .elevator
                                    .getCoralIntakeEjectFinalPose()
                                    .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                                () ->
                                    RobotContainer.model
                                        .elevator
                                        .getCoralIntakeChannelPose()
                                        .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                                p ->
                                    coral.poseSupplier =
                                        () ->
                                            new Pose3d(RobotContainer.model.getRobot())
                                                .plus(
                                                    new Transform3d(
                                                        p.getTranslation(), p.getRotation())),
                                0.15))
                        .andThen(
                            new PoseAnimator(
                                RobotContainer.model
                                    .elevator
                                    .getCoralIntakeChannelPose()
                                    .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                                () ->
                                    RobotContainer.model
                                        .elevatorArm
                                        .getCoralShooterTargetPose()
                                        .relativeTo(new Pose3d(RobotContainer.model.getRobot())),
                                p ->
                                    coral.poseSupplier =
                                        () ->
                                            new Pose3d(RobotContainer.model.getRobot())
                                                .plus(
                                                    new Transform3d(
                                                        p.getTranslation(), p.getRotation())),
                                0.4))
                        .andThen(
                            () -> {
                              // set has elevator coral (NC)
                              coral.poseSupplier =
                                  () ->
                                      RobotContainer.model.elevatorArm.getCoralShooterTargetPose();
                              try {
                                RobotContainer.elevatorHead.getSimIO().setCoralDetectorSim(false);
                              } catch (Exception e) {
                                e.printStackTrace();
                              }
                            })),
            new HashSet<Subsystem>()));
  }

  @Override
  public Command simulateFor(Command command) {
    if (Constants.RobotState.getMode() == Mode.REAL) {
      return command;
    } else {
      return command.alongWith(this);
    }
  }
}
