package frc.robot.utils;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class ModuleConstants {

    public int driveMotorChannel;
    public int turningMotorChannel;
    public int absoluteTurningMotorEncoderChannel;
    public double turningEncoderOffset;

    public Translation2d wheelLocation;

    public double TURN_GEAR_RATIO;
    public double DRIVE_GEAR_RATIO;

    public double TURN_KP;
    public double TURN_KI;
    public double TURN_KD;

    public double DRIVE_KP;
    public double DRIVE_KI;
    public double DRIVE_KD;

    public double DRIVE_FEEDFORWARD_KS;
    public double DRIVE_FEEDFORWARD_KV;

    public double TurnMaxAngularVelocity;
    public double TurnMaxAngularAcceleration; 
    
    public double DriveMaxAngularVelocity;
    public double DriveMaxAngularAcceleration;

    public double RELATIVE_ENCODER_RATIO;
    public double WHEEL_DIAMETER;

    // Class to store types of motors
    public enum MotorType {Falcon, Kraken}

    /**
     * essentially serves as a storage unit for one swerve module, storing every single constant that a module might want to use
     * @param driveMotorChannel drive motor port
     * @param turningMotorChannel turn motor port
     * @param absoluteTurningMotorEncoderChannel abs turn motor encoder port
     * @param turningEncoderOffset offset of the abs turn encoder at a set starting position (which we found through manually testing)
     * @param wheelLocation Translation2d object of where the wheel is relative to robot frame
     * @param turnMotorType The type of the turn motor
     * @param driveMotorType The type of the drive motor
     */

    public ModuleConstants (

        int driveMotorChannel,
        int turningMotorChannel,
        int absoluteTurningMotorEncoderChannel,
        double turningEncoderOffset,
        Translation2d wheelLocation,

        MotorType turnMotorType,
        MotorType driveMotorType) {

            // Encoder nums
            this.driveMotorChannel = driveMotorChannel;
            this.turningMotorChannel = turningMotorChannel;
            this.absoluteTurningMotorEncoderChannel = absoluteTurningMotorEncoderChannel;
            this.turningEncoderOffset = turningEncoderOffset;

            // Wheel location
            this.wheelLocation = wheelLocation; //TODO: wheel location may be motor type dependent
            
            // Max Angular Acceleration & Velocity
            this.TurnMaxAngularVelocity = Constants.Swerve.TurnMaxAngularVelocity;
            this.TurnMaxAngularAcceleration = Constants.Swerve.TurnMaxAngularAcceleration;
            this.DriveMaxAngularVelocity = Constants.Swerve.DriveMaxAngularVelocity;
            this.DriveMaxAngularAcceleration = Constants.Swerve.DriveMaxAngularAcceleration;

            // Shared miscellaneous variables
            this.RELATIVE_ENCODER_RATIO = Constants.Swerve.RELATIVE_ENCODER_RATIO;
            this.WHEEL_DIAMETER = Constants.Swerve.WHEEL_DIAMETER;

            // Turn Motor Constants
            switch(turnMotorType){
                case Falcon:
                    this.TURN_KP = Constants.Swerve.FALCON_TURN_KP;
                    this.TURN_KI = Constants.Swerve.FALCON_TURN_KI;
                    this.TURN_KD = Constants.Swerve.FALCON_TURN_KD;
                    this.TURN_GEAR_RATIO = Constants.Swerve.FALCON_TURN_GEAR_RATIO;
                    break;
                case Kraken:
                    this.TURN_KP = Constants.Swerve.KRAKEN_TURN_KP;
                    this.TURN_KI = Constants.Swerve.KRAKEN_TURN_KI;
                    this.TURN_KD = Constants.Swerve.KRAKEN_TURN_KD;
                    this.TURN_GEAR_RATIO = Constants.Swerve.KRAKEN_TURN_GEAR_RATIO;
                    break;
            }

            // Drive Motor Constants
            switch(driveMotorType){
                case Falcon:
                    this.DRIVE_KP = Constants.Swerve.FALCON_DRIVE_KP;
                    this.DRIVE_KI = Constants.Swerve.FALCON_DRIVE_KI;
                    this.DRIVE_KD = Constants.Swerve.FALCON_DRIVE_KD;
                    this.DRIVE_FEEDFORWARD_KS = Constants.Swerve.FALCON_DRIVE_FEEDFORWARD_KS;
                    this.DRIVE_FEEDFORWARD_KV = Constants.Swerve.FALCON_DRIVE_FEEDFORWARD_KV;
                    this.DRIVE_GEAR_RATIO = Constants.Swerve.FALCON_DRIVE_GEAR_RATIO;
                    break;
                case Kraken:
                    this.DRIVE_KP = Constants.Swerve.KRAKEN_DRIVE_KP;
                    this.DRIVE_KI = Constants.Swerve.KRAKEN_DRIVE_KI;
                    this.DRIVE_KD = Constants.Swerve.KRAKEN_DRIVE_KD;
                    this.DRIVE_FEEDFORWARD_KS = Constants.Swerve.KRAKEN_DRIVE_FEEDFORWARD_KS;
                    this.DRIVE_FEEDFORWARD_KV = Constants.Swerve.KRAKEN_DRIVE_FEEDFORWARD_KV;
                    this.DRIVE_GEAR_RATIO = Constants.Swerve.KRAKEN_DRIVE_GEAR_RATIO;
                    break;
            }
    }

}
