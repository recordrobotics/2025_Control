package frc.robot.commands.hybrid;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Photosensor;
import frc.robot.subsystems.Vision;
import frc.robot.utils.DriveCommandData;
/* 
public class SuperNote extends Command {
    
    Drivetrain driveTrain;
    Vision vision;
    PIDController anglePID;
    Photosensor photosensor;

    public SuperNote(Drivetrain drivetrain, Vision vision, Photosensor photosensor){
        addRequirements(drivetrain);
        setSubsystem(drivetrain.getName());
        this.driveTrain = drivetrain;
        this.vision = vision;
        this.photosensor = photosensor;

        anglePID = new PIDController(2, 0, 0);
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double angle = vision.ringDirection().getRadians();
        Translation2d speed = new Translation2d(speed * Math.cos(angle), speed * Math.sin(angle));
        driveTrain.drive(new DriveCommandData(0, 0, anglePID.calculate(angle), true));
    }

    @Override
    public boolean isFinished(){
        return photosensor.getDebouncedValue();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }

}
*/ 
