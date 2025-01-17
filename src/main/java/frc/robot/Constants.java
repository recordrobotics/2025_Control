// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.MotorLocation;
import frc.robot.utils.ModuleConstants.MotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class GroundAlgae {
    public static final double DEFAULT_SPEED = 0.1; // TODO tune
  }

  public enum ElevatorHeight {
    INTAKE(0),
    L1(0.03),
    L2(0.07),
    L3(0.11),
    L4(0.17),
    OFF(0);

    private double height;

    private ElevatorHeight(double heightMeters) {
      this.height = heightMeters;
    }

    public double getHeight() {
      return height;
    }
  }

  public enum FieldPosition {
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
    AutoStart(
        new Pose2d(15.305, 4.001, new Rotation2d(Math.PI)),
        new Pose2d(1.235, 4.001, new Rotation2d(0)));

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
    None("");

    public final String pathref;

    AutoName(String pathplannerRef) {
      pathref = pathplannerRef;
    }
  }

  public final class Elevator {
    public static final double kDt = 0.02;
    public static final double kMaxVelocity = 1.00;
    public static final double kMaxAcceleration = 0.75;
    public static final double kP = 1.3;
    public static final double kI = 0.0;
    public static final double kD = 0.7;
    public static final double kS = 1.1;
    public static final double kG = 1.2;
    public static final double kV = 1.3;
    public static final double kA = 0.2;

    public static final double DIFFERENCE_P = 0.1;
    public static final double DIFFERENCE_I = 0.0;
    public static final double DIFFERENCE_D = 0.0;

    public static final double METERS_PER_ROTATION = 1;

    public static final double AT_GOAL_POSITION_TOLERANCE = 0.01; // TODO test different values
    public static final double AT_GOAL_VELOCITY_TOLERANCE = 0.05; // TODO test different values
  }

  public final class CoralShooter {
    public static final double kP = 0.07;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.12;
    public static final double kV = 0.14;

    public static final double OUT_SPEED = 0.2; // TODO this is probably too slow
    public static final double INTAKE_SPEED = -0.2; // TODO ^^^^^^^^^^^

    public static final double SHOOT_TIME = 0.1; // TODO make correct
  }

  public final class CoralIntake {
    // TODO: need correct values
    public static final double kP = 0.07;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.12;
    public static final double kV = 0.14;

    public static final double MAX_SERVO_VELOCITY = 0.1; // TODO do the thing
    public static final double MAX_SERVO_ACCELERATION = 0.1; // TODO do the thing

    public static final double REVERSE_SPEED = 0.2; // TODO this is probably too slow
    public static final double INTAKE_SPEED = -0.2; // TODO ^^^^^^^^^^^

    public static final double INTAKE_TIME = 0.1; // TODO make correct

    public static final double sP = 0;
    public static final double sI = 0;
    public static final double sD = 0;
    public static final double sS = 0;
    public static final double sG = 0;
    public static final double sV = 0;
    public static final double sA = 0;

    public static final double SERVO_UP = Math.PI / 2;
    public static final double SERVO_DOWN = 0;
  }

  public final class Lights {
    public static final int length = 150;

    public static final double multiplier = 100; // In precent

    public static final int pulsateFrequency = 1; // Seconds
  }

  public final class FieldConstants {

    // Field width and length
    public static final double FIELD_X_DIMENSION = 17.548; // Length
    public static final double FIELD_Y_DIMENSION = 8.052; // Width
  }

  public final class Control {

    // Sensitivity for speed meter
    public static final double DIRECTIONAL_SPEED_METER_LOW = 0.25;
    public static final double DIRECTIONAL_SPEED_METER_HIGH = 4.7;
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
  }

  public final class Frame {

    /**
     * Distance between wheels (width aka between left and right and length aka between front and
     * back). Used for calculating wheel locations on the robot
     */
    public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.5969;

    public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.5969;
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
    public static final double FALCON_TURN_KP = 0.1;
    public static final double FALCON_TURN_KI = 0;
    public static final double FALCON_TURN_KD = 0;

    public static final double FALCON_DRIVE_KP = 0.2850;
    public static final double FALCON_DRIVE_KI = 0;
    public static final double FALCON_DRIVE_KD = 0;

    public static final double FALCON_DRIVE_FEEDFORWARD_KS = 0.274308;
    public static final double FALCON_DRIVE_FEEDFORWARD_KV = 2.60;
    public static final double FALCON_DRIVE_FEEDFORWARD_KA = 0.13;

    public static final double FALCON_TURN_FEEDFORWARD_KS = 0.1;
    public static final double FALCON_TURN_FEEDFORWARD_KV = 1.0;
    public static final double FALCON_TURN_FEEDFORWARD_KA = 0;

    public static final double KRAKEN_TURN_KP = 0.1;
    public static final double KRAKEN_TURN_KI = 0;
    public static final double KRAKEN_TURN_KD = 0;

    public static final double KRAKEN_DRIVE_KP = 0.2681;
    public static final double KRAKEN_DRIVE_KI = 0;
    public static final double KRAKEN_DRIVE_KD = 0;

    public static final double KRAKEN_DRIVE_FEEDFORWARD_KS = 0.1586;
    public static final double KRAKEN_DRIVE_FEEDFORWARD_KV = 2.4408;
    public static final double KRAKEN_DRIVE_FEEDFORWARD_KA = 0.1;

    public static final double KRAKEN_TURN_FEEDFORWARD_KS = 0.1;
    public static final double KRAKEN_TURN_FEEDFORWARD_KV = 1.0;
    public static final double KRAKEN_TURN_FEEDFORWARD_KA = 0;

    // Wheel diameter
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

    // Turn & Drive max velocity and acceleration
    public static final double TurnMaxAngularVelocity = 20; // ROTATIONS / SECOND
    public static final double TurnMaxAngularAcceleration = 100; // ROTATIONS / SECOND / SECOND

    /** The max speed the robot is allowed to travel */
    public static final double robotMaxSpeed = 4.7;

    /** The max jerk of the robot below which the pose is certain (in G/s) */
    public static final double MaxPoseCertaintyJerk = 80;

    public static final RobotConfig PPDefaultConfig =
        new RobotConfig(
            29.1,
            0.29,
            new ModuleConfig(0.048, 5.45, 1.2, DCMotor.getFalcon500(1), 100, 500),
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation);

    public static final PPHolonomicDriveController PPDriveController =
        new PPHolonomicDriveController(
            new PIDConstants(10, 0.0, 0.8), // Translation PID constants
            new PIDConstants(6, 0.0, 0.0) // Rotation PID constants
            );

    // Module Creation

    /**
     * ===================== NOTE !!!!!! ======================== THESE ARE BACKUP CONSTANTS - NOT
     * USED IF EVERYTHING WORKS EDIT deploy/swerve/motors.json instead
     */

    // #region BACKUP
    public static final ModuleConstants BACKUP_frontLeftConstants =
        new ModuleConstants(2, 1, 1, 0.633, frontLeftLocation, MotorType.Falcon, MotorType.Falcon);

    public static final ModuleConstants BACKUP_frontRightConstants =
        new ModuleConstants(4, 3, 2, 0.848, frontRightLocation, MotorType.Falcon, MotorType.Falcon);
    public static final ModuleConstants BACKUP_backLeftConstants =
        new ModuleConstants(8, 7, 4, 0.857, backLeftLocation, MotorType.Falcon, MotorType.Falcon);
    public static final ModuleConstants BACKUP_backRightConstants =
        new ModuleConstants(6, 5, 3, 0.554, backRightLocation, MotorType.Falcon, MotorType.Falcon);
    // #endregion

    public static final ModuleConstants frontLeftConstants =
        ModuleConstants.fromConfig(MotorLocation.FrontLeft, MotorType.Falcon);

    public static final ModuleConstants frontRightConstants =
        ModuleConstants.fromConfig(MotorLocation.FrontRight, MotorType.Falcon);

    public static final ModuleConstants backLeftConstants =
        ModuleConstants.fromConfig(MotorLocation.BackLeft, MotorType.Falcon);

    public static final ModuleConstants backRightConstants =
        ModuleConstants.fromConfig(MotorLocation.BackRight, MotorType.Falcon);
  }

  public final class RobotState {
    public static Mode getMode() {
      return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    }

    public static final boolean MOTOR_LOGGING_ENABLED = false;

    public static enum Mode {
      REAL,
      REPLAY
    }
  }
}
