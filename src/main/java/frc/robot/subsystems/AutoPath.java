package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.PushAmp;
import frc.robot.commands.auto.PushSpeaker;
import frc.robot.commands.notes.AcquireSmart;
import frc.robot.subsystems.Crashbar.CrashbarStates;
import frc.robot.subsystems.Shooter.ShooterStates;
import frc.robot.utils.DriveCommandData;

public class AutoPath {

    private final SendableChooser<Command> autoChooser;

    public AutoPath(Drivetrain drivetrain, 
                    Acquisition acquisition, 
                    Photosensor photosensor, 
                    Channel channel,
                    Shooter shooter, 
                    Crashbar crashbar) {


        // Registering named commands
        NamedCommands.registerCommand("Stop", new InstantCommand(() -> drivetrain.stop()));
        NamedCommands.registerCommand("PushSpeaker", new PushSpeaker(channel, shooter));
        NamedCommands.registerCommand("FlywheelSpeaker", new InstantCommand(() -> shooter.toggle(ShooterStates.SPEAKER)));
        NamedCommands.registerCommand("PushAmp", new PushAmp(channel, shooter, crashbar));
        NamedCommands.registerCommand("FlywheelAmp", new InstantCommand(() -> {
            shooter.toggle(ShooterStates.AMP);
            crashbar.toggle(CrashbarStates.EXTENDED);
        }));
        NamedCommands.registerCommand("Acquire", new AcquireSmart(acquisition, channel, photosensor, shooter));


        // Configures auto builder
        AutoBuilder.configureHolonomic(
                drivetrain.poseFilter::getEstimatedPosition, // Robot pose supplier
                drivetrain::setToPose, // Method to reset odometry (will be called if your auto has a starting pose)
                drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                
                // Method that will drive the robot given ROBOT RELATIVE speeds
                (speeds) -> {
                    SmartDashboard.putNumberArray("speeds",
                            new double[] { speeds.vxMetersPerSecond, speeds.vyMetersPerSecond });
                    drivetrain
                            .drive(new DriveCommandData(speeds.vxMetersPerSecond,
                                    speeds.vyMetersPerSecond,
                                    speeds.omegaRadiansPerSecond, false));
                },

                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                Constants.Swerve.PathFollowerConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                
                // Reference to this subsystem to set requirements
                drivetrain
        );
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    public void putAutoChooser() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutoChooserSelected() {
        return autoChooser.getSelected();
    }
}