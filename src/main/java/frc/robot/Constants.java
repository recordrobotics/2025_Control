// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.MotorLocation;
import frc.robot.utils.ModuleConstants.MotorType;
import frc.robot.utils.SimpleMath;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class Shooter {
    public static final double SPEAKER_SPEED = 90;
    public static final double AMP_SPEED = 60;
    public static final double REVERSE_SPEED = -30;
  }

  public final class NeoSim {
    public static final int NEO_MOTOR_KV = 473;
  }

  public final class Channel {
    public static final double SHOOT_SPEED = 1;
    public static final double THROUGH_SPEED = 1;
    public static final double REVERSE_SPEED = -1;
  }

  public final class Acquisition {
    /** Constant; The speed and direction of the acquisition on intake */
    public static final double ACQUISITION_SPEED = 1;
  }

  public enum FieldPosition {
    Speaker(Constants.FieldConstants.TEAM_RED_SPEAKER, Constants.FieldConstants.TEAM_BLUE_SPEAKER),
    Amp(Constants.FieldConstants.TEAM_RED_AMP, Constants.FieldConstants.TEAM_BLUE_AMP),
    CenterChain(
        SimpleMath.MirrorLocation(Constants.FieldConstants.TEAM_BLUE_CENTER_CHAIN),
        Constants.FieldConstants.TEAM_BLUE_CENTER_CHAIN),
  // SpeakerSideChain(SimpleMath.MirrorLocation(Constants.FieldConstants.TEAM_BLUE_SPEAKER_SIDE_CHAIN), Constants.FieldConstants.TEAM_BLUE_SPEAKER_SIDE_CHAIN),
  // AmpSideChain(SimpleMath.MirrorLocation(Constants.FieldConstants.TEAM_BLUE_AMP_SIDE_CHAIN),
  // Constants.FieldConstants.TEAM_BLUE_AMP_SIDE_CHAIN),
  // FarSideChain(SimpleMath.MirrorLocation(Constants.FieldConstants.TEAM_BLUE_FAR_SIDE_CHAIN),
  // Constants.FieldConstants.TEAM_BLUE_FAR_SIDE_CHAIN)
  ;

    private Translation2d red;
    private Translation2d blue;

    private FieldPosition(Translation2d red, Translation2d blue) {
      this.red = red;
      this.blue = blue;
    }

    public Translation2d getPose() {
      if (DriverStationUtils.getCurrentAlliance() == Alliance.Red) return red;
      else return blue;
    }
  }

  public enum FieldStartingLocation {
    FrontSpeaker(
        // Red
        new Pose2d(15.214, 5.584, Rotation2d.fromDegrees(180)),
        // Blue
        new Pose2d(2.371, 5.584, Rotation2d.fromDegrees(0))),
    FrontSpeakerClose(
        // Red
        new Pose2d(15.214, 5.584, Rotation2d.fromDegrees(180)),
        // Blue
        new Pose2d(2.371, 5.584, Rotation2d.fromDegrees(0))),
    AtAmp(
        // Red
        new Pose2d(14.89, 7.27, Rotation2d.fromDegrees(-90)),
        // Blue
        new Pose2d(1.65, 7.27, Rotation2d.fromDegrees(-90))),
    DiagonalSpeaker(
        // Red
        new Pose2d(16.268, 4.454, Rotation2d.fromDegrees(-120.665)),
        // Blue
        new Pose2d(1.564, 4.403, Rotation2d.fromDegrees(-60.849))),
    DiaAmpSpeaker(
        // Red
        new Pose2d(16.199, 6.730, Rotation2d.fromDegrees(119.8)),
        // Blue
        new Pose2d(1.776, 6.703, Rotation2d.fromDegrees(58.762))),

    SourceStart(
        // Red
        new Pose2d(15.15, 4.22, Rotation2d.fromDegrees(180)),
        // Blue
        new Pose2d(1.39, 4.22, Rotation2d.fromDegrees(0))),
    AmpStart(
        // Red
        new Pose2d(15.15, 6.85, Rotation2d.fromDegrees(180)),
        // Blue
        new Pose2d(1.39, 6.85, Rotation2d.fromDegrees(0)));

    private final Pose2d m_transformRed;
    private final Pose2d m_transformBlue;

    private FieldStartingLocation(Pose2d poseRed, Pose2d poseBlue) {
      m_transformRed = poseRed;
      m_transformBlue = poseBlue;
    }

    public Pose2d getPose() {
      return DriverStationUtils.getCurrentAlliance() == Alliance.Red
          ? m_transformRed
          : m_transformBlue;
    }
  }

  // Auto routines
  public enum AutoName {
    // Center Note Routines
    _2NoteSpeaker("2NoteSpeaker"),
    _3NoteAmpSide("3NoteAmpSide"),
    _3NoteSpeakerSide("3NoteSpeakerSide"),
    _4NoteSpeaker("4NoteSpeaker"),
    UnderStage3Note("UnderStage3Note"),
    _4NoteBlitz("4NoteBlitz"),
    // Amp Side Routines
    AmpSideDiag1("AmpSideDiag1"),
    AmpSideDiagTaxi("AmpSideDiagTaxi"),
    AmpSideInner2Note("AmpSideInner2Note"),
    DiagJustShoot("DiagJustShoot"),
    // Speaker Side Routines
    SourceSide1Note("SourceSide1Note"),
    SourceSide2Note("SourceSide2Note"),
    SourceSide3Note("SourceSide3Note"),
    SourceSideInner2Note("SourceSideInner2Note"),
    FarSpeaker2Note("FarSpeaker2Note"),
    FarSpeaker("FarSpeaker"),
    SourceSideScramble("SourceSideScramble"),

    MeterTwo("metertwoauto"),
    Rotate("rotateauto");

    public final String pathref;

    AutoName(String pathplannerRef) {
      pathref = pathplannerRef;
    }
  }

  public final class FieldConstants {

    public static final Translation2d TEAM_RED_SPEAKER = new Translation2d(16, 5.5);
    public static final Translation2d TEAM_BLUE_SPEAKER = new Translation2d(0.6, 5.6);
    public static final Translation2d TEAM_RED_AMP = new Translation2d(14.7, 8.5);
    public static final Translation2d TEAM_BLUE_AMP = new Translation2d(2.775, 8.5);

    public static final Translation2d TEAM_BLUE_CENTER_CHAIN = new Translation2d(4.89, 4.09);

    // Field width and length
    public static final double FIELD_X_DIMENSION = 16.54; // Length
    public static final double FIELD_Y_DIMENSION = 8.21; // Width
  }

  public final class Control {

    // Sensitivity for speed meter
    public static final double DIRECTIONAL_SPEED_METER_LOW = 0.25;
    public static final double DIRECTIONAL_SPEED_METER_HIGH = 4.0;
    public static final double SPIN_SPEED_METER_LOW = 0.5;
    public static final double SPIN_SPEED_METER_HIGH = 2.4;

    // Sensitivies for directional controls (XY) and spin (theta)
    public static final double JOSYSTICK_DIRECTIONAL_SENSITIVITY = 1;
    public static final double JOYSTICK_SPIN_SENSITIVITY = 2;
    public static final double JOYSTICK_X_THRESHOLD = 0.15;
    public static final double JOYSTICK_Y_THRESHOLD = 0.15;
    public static final double JOYSTICK_SPIN_THRESHOLD = 0.3;

    // Thresholds for directional controls (XY) and spin (theta)
    public static final double XBOX_DIRECTIONAL_SENSITIVITY = 1;
    public static final double XBOX_X_THRESHOLD = 0.15;
    public static final double XBOX_Y_THRESHOLD = 0.15;
    public static final double XBOX_SPIN_THRESHOLD = 0.3;

    public static final double XBOX_SPIN_ROT_THRESHOLD = 0.1;
    public static final double XBOX_SPIN_ROT_SENSITIVITY = 1.0;

    // Tablet drive constants
    public final class Tablet {
      // Will fill in later, but for now it's convenient to have it in the TabletDrive
      public static final double PRESSURE_THRESHOLD = 0.2;
      public static final double MIN_SPEED = 0.2;
      public static final double STEEPNESS =
          2.6; // Linear = 1, <1 = faster scaling, >1 = slower scaling
    }
  }

  public final class Frame {

    /**
     * Distance between wheels (width aka between left and right and length aka between front and
     * back). Used for calculating wheel locations on the robot
     */
    public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.46;

    public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.46;
  }

  public final class Swerve {

    // Works out module locations
    private static final double locX = Frame.ROBOT_WHEEL_DISTANCE_WIDTH / 2;
    private static final double locY = Frame.ROBOT_WHEEL_DISTANCE_LENGTH / 2;
    public static final double locDist = Math.sqrt(locX * locX + locY * locY);

    private static final Translation2d frontLeftLocation = new Translation2d(locX, locY);
    private static final Translation2d frontRightLocation = new Translation2d(locX, -locY);
    private static final Translation2d backLeftLocation = new Translation2d(-locX, locY);
    private static final Translation2d backRightLocation = new Translation2d(-locX, -locY);

    // Gear ratios for falcon and kraken
    public static final double FALCON_TURN_GEAR_RATIO =
        15.43; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)
    public static final double FALCON_DRIVE_GEAR_RATIO =
        7.36; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)

    public static final double KRAKEN_TURN_GEAR_RATIO = 13.3714;
    public static final double KRAKEN_DRIVE_GEAR_RATIO = 6.75; // X1 12 pinion

    // PID Values
    public static final double FALCON_TURN_KP = 1;
    public static final double FALCON_TURN_KI = 0;
    public static final double FALCON_TURN_KD = 0;

    public static final double FALCON_DRIVE_KP = 0.2681;
    public static final double FALCON_DRIVE_KI = 0;
    public static final double FALCON_DRIVE_KD = 0;

    public static final double FALCON_DRIVE_FEEDFORWARD_KS = 0.1586;
    public static final double FALCON_DRIVE_FEEDFORWARD_KV = 2.4408;

    public static final double KRAKEN_TURN_KP = 2.3;
    public static final double KRAKEN_TURN_KI = 0;
    public static final double KRAKEN_TURN_KD = 0;

    public static final double KRAKEN_DRIVE_KP = 0.2681;
    public static final double KRAKEN_DRIVE_KI = 0;
    public static final double KRAKEN_DRIVE_KD = 0;

    public static final double KRAKEN_DRIVE_FEEDFORWARD_KS = 0.1586;
    public static final double KRAKEN_DRIVE_FEEDFORWARD_KV = 2.4408;

    // Same between Falcon and Kraken since they share the same encoders
    public static final double RELATIVE_ENCODER_RATIO = 2048;

    // Wheel diameter
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

    // Turn & Drive max velocity and acceleration
    public static final double TurnMaxAngularVelocity = 25; // Drivetrain.kMaxAngularSpeed;
    public static final double TurnMaxAngularAcceleration =
        34; // 2 * Math.PI; // radians per second squared
    public static final double DriveMaxAngularVelocity = 15; // Drivetrain.kMaxAngularSpeed;
    public static final double DriveMaxAngularAcceleration =
        30; // 2 * Math.PI; // radians per second squared

    /** The max speed the robot is allowed to travel */
    public static final double robotMaxSpeed = 7.0;

    /** The max jerk of the robot below which the pose is certain (in G/s) */
    public static final double MaxPoseCertaintyJerk = 80;

    public static final RobotConfig PPDefaultConfig = new RobotConfig(1, 1, null, 1);

    public static final PPHolonomicDriveController PPDriveController =
        new PPHolonomicDriveController(
            new PIDConstants(6, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
            );

    // Module Creation

    /**
     * ===================== NOTE !!!!!! ======================== THESE ARE BACKUP CONSTANTS - NOT
     * USED IF EVERYTHING WORKS EDIT deploy/swerve/motors.json instead
     */

    // #region BACKUP
    public static final ModuleConstants BACKUP_frontLeftConstants =
        new ModuleConstants(2, 1, 1, 0.633, frontLeftLocation, MotorType.Kraken, MotorType.Kraken);

    public static final ModuleConstants BACKUP_frontRightConstants =
        new ModuleConstants(4, 3, 2, 0.848, frontRightLocation, MotorType.Kraken, MotorType.Kraken);
    public static final ModuleConstants BACKUP_backLeftConstants =
        new ModuleConstants(8, 7, 4, 0.857, backLeftLocation, MotorType.Kraken, MotorType.Kraken);
    public static final ModuleConstants BACKUP_backRightConstants =
        new ModuleConstants(6, 5, 3, 0.554, backRightLocation, MotorType.Kraken, MotorType.Kraken);
    // #endregion

    public static final ModuleConstants frontLeftConstants =
        ModuleConstants.fromConfig(MotorLocation.FrontLeft, MotorType.Kraken);

    public static final ModuleConstants frontRightConstants =
        ModuleConstants.fromConfig(MotorLocation.FrontRight, MotorType.Kraken);

    public static final ModuleConstants backLeftConstants =
        ModuleConstants.fromConfig(MotorLocation.BackLeft, MotorType.Kraken);

    public static final ModuleConstants backRightConstants =
        ModuleConstants.fromConfig(MotorLocation.BackRight, MotorType.Kraken);
  }

  public final class Vision {

    public static final String cameraID = new String("photonvision");

    // The offset from the center of the robot to the camera, and from facing exactly forward to the
    // orientation of the camera.
    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(11), -1 * Units.inchesToMeters(9), 0.1725),
            new Rotation3d(0, 0, 0));

    public static final Transform3d[]
        tagTransforms = { // april tags 1-8 in order. values contained are x, y, z, theta, in that
      // order. x, y, z are distances in meters, theta is in radians.
      new Transform3d(
          new Translation3d(15.513558, 1.071626, 0.462788), new Rotation3d(0, 0, Math.PI)),
      new Transform3d(
          new Translation3d(15.513558, 2.748026, 0.462788), new Rotation3d(0, 0, Math.PI)),
      new Transform3d(
          new Translation3d(15.513558, 4.424426, 0.462788), new Rotation3d(0, 0, Math.PI)),
      new Transform3d(
          new Translation3d(16.178784, 6.749796, 0.695452), new Rotation3d(0, 0, Math.PI)),
      new Transform3d(new Translation3d(0.36195, 6.749796, 0.695452), new Rotation3d(0, 0, 0)),
      new Transform3d(new Translation3d(1.8415, 8.2042, 1.355852), new Rotation3d(0, 0, 4.71239)),
      new Transform3d(new Translation3d(1.02743, 2.748026, 0.462788), new Rotation3d(0, 0, 0)),
      new Transform3d(new Translation3d(1.02743, 1.071626, 0.462788), new Rotation3d(0, 0, 0))
    };
  }
}
