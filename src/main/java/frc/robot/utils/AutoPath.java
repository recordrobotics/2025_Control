package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.notes.Acquire;
import frc.robot.commands.notes.AcquireSmart;
import frc.robot.commands.notes.PushAcquire;
import frc.robot.commands.notes.RetractAcquire;
import frc.robot.commands.subroutines.PushAmp;
import frc.robot.commands.subroutines.PushSpeaker;
import frc.robot.commands.subroutines.SetupAmp;
import frc.robot.commands.subroutines.SetupSpeaker;

public class AutoPath {

  public AutoPath() {

    // Registering named commands (so that the pathplanner can call them by name)

    // Stop the robot's movement
    NamedCommands.registerCommand(
        "Stop", new InstantCommand(() -> RobotContainer.drivetrain.kill()));

    // Acqire a note
    NamedCommands.registerCommand("Acquire", new AcquireSmart());

    // Acqire a note stupidly (depreciated) (legacy)
    NamedCommands.registerCommand(
        "AcquireStupid",
        new Acquire(
            RobotContainer.acquisition, RobotContainer.channel, RobotContainer.photosensor));

    // Turn acquisition and channel on
    NamedCommands.registerCommand(
        "AcquirePush", new PushAcquire(RobotContainer.acquisition, RobotContainer.channel));

    // Turn acquisition off and channel reverse for 0.3 seconds
    NamedCommands.registerCommand(
        "Retract", new RetractAcquire(RobotContainer.acquisition, RobotContainer.channel));

    // Assumes flywheel is already on and shoots
    NamedCommands.registerCommand(
        "PushSpeaker", new PushSpeaker(RobotContainer.channel, RobotContainer.shooter));

    // Turns shooter on
    NamedCommands.registerCommand("FlywheelSpeaker", new SetupSpeaker(RobotContainer.shooter));

    // Assumes flywheel and crashbar are ready and shoots amp
    NamedCommands.registerCommand(
        "PushAmp",
        new PushAmp(RobotContainer.channel, RobotContainer.shooter, RobotContainer.crashbar));

    // Prepare for PushAmp
    NamedCommands.registerCommand(
        "FlywheelAmp", new SetupAmp(RobotContainer.shooter, RobotContainer.crashbar, true));

    // Configures auto builder
    AutoBuilder.configureHolonomic(
        RobotContainer.poseTracker::getEstimatedPosition, // Robot pose supplier
        RobotContainer.poseTracker
            ::setToPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        RobotContainer.drivetrain
            ::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

        // Method that will drive the robot given ROBOT RELATIVE speeds
        (speeds) -> {
          RobotContainer.drivetrain.drive(
              new DriveCommandData(
                  speeds.vxMetersPerSecond,
                  speeds.vyMetersPerSecond,
                  speeds.omegaRadiansPerSecond,
                  false));
        },

        // com.pathplanner.lib.util.HolonomicPathFollowerConfig for configuring the path following
        // commands
        Constants.Swerve.PathFollowerConfig,

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
  }
}
