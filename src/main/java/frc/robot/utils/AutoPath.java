package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.Align;
import frc.robot.commands.CoralIntakeFromSource;
import frc.robot.commands.CoralIntakeMoveL1;
import frc.robot.commands.CoralIntakeShootL1;
import frc.robot.commands.CoralShoot;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.ElevatorMoveThenAlgaeGrab;
import frc.robot.commands.ElevatorMoveThenAlgaeGrabEnd;
import frc.robot.commands.ProcessorScore;
import frc.robot.utils.libraries.Elastic.Notification.NotificationLevel;
import org.littletonrobotics.junction.Logger;

public class AutoPath {

  public AutoPath() {
    RobotConfig config = Constants.Swerve.PPDefaultConfig;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      Notifications.send(
          NotificationLevel.ERROR, "AutoPath failed to load config", "Error message in console");
    }

    // Registering named commands (so that the pathplanner can call them by name)

    // Stop the robot's movement
    NamedCommands.registerCommand(
        "Stop", new InstantCommand(() -> RobotContainer.drivetrain.kill()));

    NamedCommands.registerCommand(
        "AutoAlign",
        CommandUtils.finishOnInterrupt(Align.create(0.01, 0.05, true).withTimeout(1.0)));
    NamedCommands.registerCommand("ElevatorL4", new ElevatorMove(ElevatorHeight.L4));
    NamedCommands.registerCommand("ElevatorL3", new ElevatorMove(ElevatorHeight.L3));
    NamedCommands.registerCommand("ElevatorL2", new ElevatorMove(ElevatorHeight.L2));
    NamedCommands.registerCommand("ElevatorDown", new ElevatorMove(ElevatorHeight.BOTTOM));
    NamedCommands.registerCommand(
        "AlgaeL2", ElevatorMoveThenAlgaeGrab.create(ElevatorHeight.LOW_REEF_ALGAE));
    NamedCommands.registerCommand(
        "AlgaeL2End", new ElevatorMoveThenAlgaeGrabEnd(ElevatorHeight.LOW_REEF_ALGAE));
    NamedCommands.registerCommand("ProcessorScore", new ProcessorScore());
    NamedCommands.registerCommand(
        "SourceIntake",
        CommandUtils.finishOnInterrupt(new CoralIntakeFromSource(false).withTimeout(4.5)));
    NamedCommands.registerCommand(
        "SourceIntakeStart",
        CommandUtils.finishOnInterrupt(new CoralIntakeFromSource(false).withTimeout(4)));
    NamedCommands.registerCommand(
        "SourceIntakeEnd",
        CommandUtils.finishOnInterrupt(new CoralIntakeFromSource(false).withTimeout(4)));
    NamedCommands.registerCommand(
        "CoralShoot", CommandUtils.finishOnInterrupt(new CoralShoot().withTimeout(2)));
    NamedCommands.registerCommand(
        "CoralL1ArmMove", CommandUtils.finishOnInterrupt(new CoralIntakeMoveL1().withTimeout(3)));
    NamedCommands.registerCommand(
        "CoralL1Shoot", CommandUtils.finishOnInterrupt(new CoralIntakeShootL1().withTimeout(3)));

    // Configures auto builder
    AutoBuilder.configure(
        RobotContainer.poseTracker::getEstimatedPosition, // Robot pose supplier
        RobotContainer.poseTracker
            ::setToPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        RobotContainer.drivetrain
            ::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

        // Method that will drive the robot given ROBOT RELATIVE speeds
        (speeds, feedforwards) -> {
          RobotContainer.drivetrain.drive(
              new DriveCommandData(
                  speeds.vxMetersPerSecond,
                  speeds.vyMetersPerSecond,
                  speeds.omegaRadiansPerSecond,
                  false));
        },
        Constants.Swerve.PPDriveController,
        config,

        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },

        // Reference to this subsystem to set requirements
        RobotContainer.drivetrain);

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
    PathfindingCommand.warmupCommand().schedule();
  }
}
