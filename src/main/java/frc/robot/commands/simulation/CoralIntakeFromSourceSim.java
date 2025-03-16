package frc.robot.commands.simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotModel.NamedCoral;
import java.util.HashSet;

public class CoralIntakeFromSourceSim extends SequentialCommandGroup implements SimulationCommand {

  private Pose3d coralPose;
  private NamedCoral coral;

  public CoralIntakeFromSourceSim() {
    if (Constants.RobotState.getMode() == Mode.REAL) return;

    addCommands(
        new WaitUntilCommand(
            () -> {
              Pose2d robotOrigin = new Pose2d();
              if (RobotContainer.poseTracker != null)
                robotOrigin = RobotContainer.poseTracker.getEstimatedPosition();

              Pose3d closestSource =
                  new Pose3d(Constants.FieldConstants.closestSourceTo(robotOrigin));

              Pose3d relativeRobot = new Pose3d(robotOrigin).relativeTo(closestSource);

              Pose3d coralPose =
                  closestSource.transformBy(
                      new Transform3d(
                          MathUtil.clamp(relativeRobot.getX() + 0.1, -0.75, 0.75),
                          0,
                          1.16,
                          new Rotation3d(0, 0, 0)));

              Pose3d intake = RobotContainer.model.elevator.getCoralIntakeEjectFinalPose();
              return intake.getTranslation().getDistance(coralPose.getTranslation()) < 0.8;
            }),
        new InstantCommand(
            () -> {
              Pose2d robotOrigin = new Pose2d();
              if (RobotContainer.poseTracker != null)
                robotOrigin = RobotContainer.poseTracker.getEstimatedPosition();

              Pose3d closestSource =
                  new Pose3d(Constants.FieldConstants.closestSourceTo(robotOrigin));

              Pose3d relativeRobot = new Pose3d(robotOrigin).relativeTo(closestSource);

              coralPose =
                  closestSource.transformBy(
                      new Transform3d(
                          MathUtil.clamp(relativeRobot.getX() + 0.1, -0.75, 0.75),
                          0,
                          1.16,
                          new Rotation3d(0, 0, 0)));

              coral = new NamedCoral("CoralIntakeToElevator/Coral", coralPose);

              RobotContainer.model.addCoral(coral);
            }),
        new DeferredCommand(
            () -> {
              Pose3d intake = RobotContainer.model.elevator.getCoralIntakeEjectFinalPose();

              double dt1 =
                  Math.abs(
                      intake.getRotation().getZ()
                          - MathUtil.angleModulus(coralPose.getRotation().getZ()));
              double dt2 =
                  Math.abs(
                      intake.getRotation().getZ()
                          - MathUtil.angleModulus(coralPose.getRotation().getZ() + Math.PI));

              if (dt2 < dt1) {
                coralPose =
                    new Pose3d(
                        coralPose.getTranslation(),
                        coralPose.getRotation().plus(new Rotation3d(0, 0, Math.PI)));
              }

              return new PoseAnimator(
                      coralPose,
                      () -> RobotContainer.model.elevator.getCoralIntakeEjectFinalPose(),
                      p -> coral.pose = () -> p,
                      0.3)
                  .andThen(new CoralIntakeToElevator());
            },
            new HashSet<>()),
        // set has elevator coral (NC)
        new InstantCommand(
            () -> {
              coral.name = "CoralShooter/Coral";
              coral.pose = () -> RobotContainer.model.elevatorArm.getCoralShooterTargetPose();
              try {
                RobotContainer.coralShooter.getSimIO().setCoralDetectorSim(false);
              } catch (Exception e) {
                e.printStackTrace();
              }
            }));
  }

  //  private int counter = 0;

  @Override
  public Command simulateFor(Command command) {
    if (Constants.RobotState.getMode() == Mode.REAL) {
      return command;
    } else {
      return command.alongWith(
          this
          // .onlyIf(()->counter++ > 2)
          );
    }
  }
}
