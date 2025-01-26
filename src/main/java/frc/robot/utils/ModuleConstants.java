package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class ModuleConstants {

  private static JSONParser parser = new JSONParser();

  public enum MotorLocation {
    FrontLeft,
    FrontRight,
    BackLeft,
    BackRight
  }

  public int driveMotorChannel;
  public int turningMotorChannel;
  public int absoluteTurningMotorEncoderChannel;
  public double turningEncoderOffset;

  public Translation2d wheelLocation;

  public double TURN_GEAR_RATIO;
  public double DRIVE_GEAR_RATIO;

  public double DRIVE_KP;
  public double DRIVE_KI;
  public double DRIVE_KD;

  public double DRIVE_FEEDFORWARD_KS;
  public double DRIVE_FEEDFORWARD_KV;
  public double DRIVE_FEEDFORWARD_KA;

  public double TURN_KV;
  public double TURN_KA;

  public double TURN_STD_STATE_POSITION;
  public double TURN_STD_STATE_VELOCITY;
  public double TURN_STD_ENCODER_POSITION;
  public double TURN_STD_ENCODER_VELOCITY;

  public double TURN_REGULATOR_POSITION_ERROR_TOLERANCE;
  public double TURN_REGULATOR_VELOCITY_ERROR_TOLERANCE;
  public double TURN_REGULATOR_CONTROL_EFFORT_TOLERANCE;

  public double TurnMaxAngularVelocity;
  public double TurnMaxAngularAcceleration;

  public double WHEEL_DIAMETER;

  // Class to store types of motors
  public enum MotorType {
    Falcon,
    Kraken
  }

  /**
   * essentially serves as a storage unit for one swerve module, storing every single constant that
   * a module might want to use
   *
   * @param driveMotorChannel drive motor port
   * @param turningMotorChannel turn motor port
   * @param absoluteTurningMotorEncoderChannel abs turn motor encoder port
   * @param turningEncoderOffset offset of the abs turn encoder at a set starting position (which we
   *     found through manually testing)
   * @param wheelLocation Translation2d object of where the wheel is relative to robot frame
   * @param turnMotorType The type of the turn motor
   * @param driveMotorType The type of the drive motor
   */
  public ModuleConstants(
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
    this.wheelLocation = wheelLocation;

    // Max Angular Acceleration & Velocity
    this.TurnMaxAngularVelocity = Constants.Swerve.TurnMaxAngularVelocity;
    this.TurnMaxAngularAcceleration = Constants.Swerve.TurnMaxAngularAcceleration;

    // Shared miscellaneous variables
    this.WHEEL_DIAMETER = Constants.Swerve.WHEEL_DIAMETER;

    // Turn Motor Constants
    switch (turnMotorType) {
      case Falcon:
        this.TURN_KV = Constants.Swerve.FALCON_TURN_KV;
        this.TURN_KA = Constants.Swerve.FALCON_TURN_KA;
        this.TURN_STD_STATE_POSITION = Constants.Swerve.FALCON_TURN_STD_STATE_POSITION;
        this.TURN_STD_STATE_VELOCITY = Constants.Swerve.FALCON_TURN_STD_STATE_VELOCITY;
        this.TURN_STD_ENCODER_POSITION = Constants.Swerve.FALCON_TURN_STD_ENCODER_POSITION;
        this.TURN_STD_ENCODER_VELOCITY = Constants.Swerve.FALCON_TURN_STD_ENCODER_VELOCITY;
        this.TURN_REGULATOR_POSITION_ERROR_TOLERANCE =
            Constants.Swerve.FALCON_TURN_REGULATOR_POSITION_ERROR_TOLERANCE;
        this.TURN_REGULATOR_VELOCITY_ERROR_TOLERANCE =
            Constants.Swerve.FALCON_TURN_REGULATOR_VELOCITY_ERROR_TOLERANCE;
        this.TURN_REGULATOR_CONTROL_EFFORT_TOLERANCE =
            Constants.Swerve.FALCON_TURN_REGULATOR_CONTROL_EFFORT_TOLERANCE;
        this.TURN_GEAR_RATIO = Constants.Swerve.FALCON_TURN_GEAR_RATIO;
        break;
      case Kraken:
        this.TURN_KV = Constants.Swerve.KRAKEN_TURN_KV;
        this.TURN_KA = Constants.Swerve.KRAKEN_TURN_KA;
        this.TURN_STD_STATE_POSITION = Constants.Swerve.KRAKEN_TURN_STD_STATE_POSITION;
        this.TURN_STD_STATE_VELOCITY = Constants.Swerve.KRAKEN_TURN_STD_STATE_VELOCITY;
        this.TURN_STD_ENCODER_POSITION = Constants.Swerve.KRAKEN_TURN_STD_ENCODER_POSITION;
        this.TURN_STD_ENCODER_VELOCITY = Constants.Swerve.KRAKEN_TURN_STD_ENCODER_VELOCITY;
        this.TURN_REGULATOR_POSITION_ERROR_TOLERANCE =
            Constants.Swerve.KRAKEN_TURN_REGULATOR_POSITION_ERROR_TOLERANCE;
        this.TURN_REGULATOR_VELOCITY_ERROR_TOLERANCE =
            Constants.Swerve.KRAKEN_TURN_REGULATOR_VELOCITY_ERROR_TOLERANCE;
        this.TURN_REGULATOR_CONTROL_EFFORT_TOLERANCE =
            Constants.Swerve.KRAKEN_TURN_REGULATOR_CONTROL_EFFORT_TOLERANCE;
        this.TURN_GEAR_RATIO = Constants.Swerve.KRAKEN_TURN_GEAR_RATIO;
        break;
    }

    // Drive Motor Constants
    switch (driveMotorType) {
      case Falcon:
        this.DRIVE_KP = Constants.Swerve.FALCON_DRIVE_KP;
        this.DRIVE_KI = Constants.Swerve.FALCON_DRIVE_KI;
        this.DRIVE_KD = Constants.Swerve.FALCON_DRIVE_KD;
        this.DRIVE_FEEDFORWARD_KS = Constants.Swerve.FALCON_DRIVE_FEEDFORWARD_KS;
        this.DRIVE_FEEDFORWARD_KV = Constants.Swerve.FALCON_DRIVE_FEEDFORWARD_KV;
        this.DRIVE_FEEDFORWARD_KA = Constants.Swerve.FALCON_DRIVE_FEEDFORWARD_KA;
        this.DRIVE_GEAR_RATIO = Constants.Swerve.FALCON_DRIVE_GEAR_RATIO;
        break;
      case Kraken:
        this.DRIVE_KP = Constants.Swerve.KRAKEN_DRIVE_KP;
        this.DRIVE_KI = Constants.Swerve.KRAKEN_DRIVE_KI;
        this.DRIVE_KD = Constants.Swerve.KRAKEN_DRIVE_KD;
        this.DRIVE_FEEDFORWARD_KS = Constants.Swerve.KRAKEN_DRIVE_FEEDFORWARD_KS;
        this.DRIVE_FEEDFORWARD_KV = Constants.Swerve.KRAKEN_DRIVE_FEEDFORWARD_KV;
        this.DRIVE_FEEDFORWARD_KA = Constants.Swerve.KRAKEN_DRIVE_FEEDFORWARD_KA;
        this.DRIVE_GEAR_RATIO = Constants.Swerve.KRAKEN_DRIVE_GEAR_RATIO;
        break;
    }
  }

  private static ModuleConstants getDefault(MotorLocation location) {
    System.out.println("[MODULE CONSTANTS] Config forced to load default for location " + location);
    switch (location) {
      case FrontLeft:
        return Constants.Swerve.BACKUP_frontLeftConstants;
      case FrontRight:
        return Constants.Swerve.BACKUP_frontRightConstants;
      case BackLeft:
        return Constants.Swerve.BACKUP_backLeftConstants;
      case BackRight:
        return Constants.Swerve.BACKUP_backRightConstants;
    }

    return null;
  }

  /**
   * Loads module constants from deploy/swerve/motors.json file
   *
   * @param location Location of module
   * @param motorType Motor type (determines the falcon/kraken objects in config)
   */
  public static ModuleConstants fromConfig(MotorLocation location, MotorType motorType) {
    return fromConfig(location, motorType, motorType);
  }

  /**
   * Loads module constants from deploy/swerve/motors.json file
   *
   * @param location Location of module
   * @param driveMotorType Drive motor type (also determines the falcon/kraken objects in config)
   * @param turnMotorType Turn motor type (only used for the final module creation)
   */
  public static ModuleConstants fromConfig(
      MotorLocation location, MotorType driveMotorType, MotorType turnMotorType) {
    File configFile = new File(Filesystem.getDeployDirectory(), "swerve/motors.json");
    if (!configFile.exists()) return getDefault(location);

    JSONObject obj;
    try {
      FileReader reader = new FileReader(configFile);
      obj = (JSONObject) parser.parse(reader);
      reader.close();
    } catch (IOException | ParseException e) {
      return getDefault(location);
    }

    JSONObject motor;
    Translation2d loc;
    switch (location) {
      case FrontLeft:
        {
          loc = Constants.Swerve.BACKUP_frontLeftConstants.wheelLocation;
          var val = obj.get("front-left");
          if (val == null) return getDefault(location);
          else motor = (JSONObject) val;
          break;
        }
      case FrontRight:
        {
          loc = Constants.Swerve.BACKUP_frontRightConstants.wheelLocation;
          var val = obj.get("front-right");
          if (val == null) return getDefault(location);
          else motor = (JSONObject) val;
          break;
        }
      case BackLeft:
        {
          loc = Constants.Swerve.BACKUP_backLeftConstants.wheelLocation;
          var val = obj.get("back-left");
          if (val == null) return getDefault(location);
          else motor = (JSONObject) val;
          break;
        }
      case BackRight:
        {
          loc = Constants.Swerve.BACKUP_backRightConstants.wheelLocation;
          var val = obj.get("back-right");
          if (val == null) return getDefault(location);
          else motor = (JSONObject) val;
          break;
        }
      default:
        return getDefault(location);
    }

    Long driveMotorChannel = (Long) motor.get("driveMotorChannel");
    if (driveMotorChannel == null) return getDefault(location);

    Long turningMotorChannel = (Long) motor.get("turningMotorChannel");
    if (turningMotorChannel == null) return getDefault(location);

    Long encoderChannel = (Long) motor.get("encoderChannel");
    if (encoderChannel == null) return getDefault(location);

    JSONObject encoderOffset = (JSONObject) motor.get("encoderOffset");
    if (encoderOffset == null) return getDefault(location);

    Double encoderOffsetVal;
    switch (driveMotorType) {
      case Falcon:
        {
          var val = encoderOffset.get("falcon");
          if (val == null) return getDefault(location);
          else encoderOffsetVal = (Double) val;
          break;
        }
      case Kraken:
        {
          var val = encoderOffset.get("kraken");
          if (val == null) return getDefault(location);
          else encoderOffsetVal = (Double) val;
          break;
        }
      default:
        return getDefault(location);
    }

    return new ModuleConstants(
        Math.toIntExact(driveMotorChannel),
        Math.toIntExact(turningMotorChannel),
        Math.toIntExact(encoderChannel),
        encoderOffsetVal,
        loc,
        turnMotorType,
        driveMotorType);
  }
}
