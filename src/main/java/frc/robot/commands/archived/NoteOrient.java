package frc.robot.commands.archived;
// package frc.robot.commands.hybrid;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Photosensor;
// import frc.robot.subsystems.Vision;
// import frc.robot.utils.DriveCommandData;

// public class NoteOrient extends Command{

//     Drivetrain driveTrain;
//     Vision vision;
//     PIDController anglePID;
//     Photosensor photosensor;
//     BooleanSupplier button;

//     public NoteOrient(Drivetrain drivetrain, Vision vision, Photosensor photosensor,
// BooleanSupplier button){
//         addRequirements(drivetrain);
//         setSubsystem(drivetrain.getName());
//         this.driveTrain = drivetrain;
//         this.vision = vision;
//         this.photosensor = photosensor;

//         anglePID = new PIDController(2, 0, 0);
//         anglePID.enableContinuousInput(-Math.PI, Math.PI);
//     }

//     @Override
//     public void initialize() {
//         driveTrain.kill();
//     }

//     @Override
//     public void execute() {
//         driveTrain.drive(new DriveCommandData(0, 0,
// anglePID.calculate(vision.ringDirection().getRadians()), false));
//     }

//     @Override
//     public boolean isFinished(){
//         return (vision.ringDirection().getRadians() < 0.1 && vision.ringDirection().getRadians()
// > -0.1) || !button.getAsBoolean();
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }
// }
