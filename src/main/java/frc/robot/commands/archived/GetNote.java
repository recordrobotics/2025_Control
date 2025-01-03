package frc.robot.commands.archived;
// package frc.robot.commands.hybrid;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Acquisition;
// import frc.robot.subsystems.Channel;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Photosensor;
// import frc.robot.subsystems.Vision;
// import frc.robot.utils.DriveCommandData;

// public class GetNote extends Command{
//     Drivetrain driveTrain;
//     Vision vision;
//     double speed;
//     Photosensor _photosensor;
//     Acquisition _acquisition;
//     Channel _channel;

//     public GetNote(Drivetrain drivetrain, Vision vision, double speed, Photosensor photosensor,
// Acquisition acquisition, Channel channel){
//         addRequirements(drivetrain);
//         addRequirements(acquisition);
//         addRequirements(channel);
//         addRequirements(photosensor);

//         this.driveTrain = drivetrain;
//         this.vision = vision;
//         this.speed = speed;

//         _acquisition = acquisition;
//         _channel = channel;
//         _photosensor = photosensor;
//     }

//     @Override
//     public void execute(){
//         driveTrain.drive(new DriveCommandData(speed, 0, 0, false));

//     }

//     @Override
//     public boolean isFinished(){
//         return _photosensor.getDebouncedValue();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveTrain.kill();
//     }
// }
