// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.CoralShooter.CoralShooterStates;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.MotorLocation;
import frc.robot.utils.ModuleConstants.MotorType;
import java.util.Map;
import java.util.function.Supplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final class HybridConstants {
    // Create the constraints to use while pathfinding. The constraints defined in the path will
    // only
    // be used for the path.
    // docs are here:
    // http://gabybot.com/RobotCoreDoc/classcom_1_1pathplanner_1_1lib_1_1path_1_1_path_constraints.html
    public static final PathConstraints constraints =
        new PathConstraints(
            0.5, // Max velocity meters per second
            4.0, // Max acceleration meters per second per second
            Units.degreesToRadians(540), // Max angular velocity radians per second
            Units.degreesToRadians(720)); // Max angular acceleration radians per second per second

    public static final double processorTriggerDistance = 3.0;
    public static final double reefTriggerDistance = 3.0;

    public static final Map<String, Boolean> isAlgaeInHighPosition =
        Map.of(
            "AB", true,
            "CD", false,
            "EF", true,
            "GH", false,
            "IJ", true,
            "KL", false);
  }

  public final class Limelight {
    public static final Angle LIMELIGHT_ANGLE_UP = Degrees.of(23); // TODO make correct
    public static final Translation3d LIMELIGHT_OFFSET =
        new Translation3d(
            Meters.of(0.2), // TODO make correct
            Meters.of(0.2), // TODO make correct
            Meters.of(0.4)); // TODO make correct

    public static final String LIMELIGHT_NAME = "limelight";
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Angle FOV_VERTICAL_FROM_CENTER = Degrees.of(28);
    public static final Angle FOV_HORIZONTAL_FROM_CENTER = Degrees.of(40);

    public static final double CROPPING_MARGIN = 0.2; // units are in ... um ... uh ... Numbers!
  }

  public final class ElevatorAlgae {
    public static final double kP = 0.07; // TODO idk where these numbers came from
    public static final double kI = 0.0; // TODO idk where these numbers came from
    public static final double kD = 0.0; // TODO idk where these numbers came from
    public static final double kS = 0.02; // TODO idk where these numbers came from
    public static final double kV = 0.14; // TODO idk where these numbers came from
    public static final double kA = 0.09; // TODO idk where these numbers came from

    public static final double OUT_SPEED = 0.2; // TODO this is probably too slow
    public static final double INTAKE_SPEED = -0.2; // TODO ^^^^^^^^^^^

    public static final double SHOOT_TIME = 0.1; // TODO make correct

    public static final double DEBOUNCE_TIME = 0.1; // TODO idk where these numbers came from
  }

  public enum ElevatorHeight {
    INTAKE(0),
    L1(0.06),
    L2(0.2),
    LOW_ALGAE(0.465),
    L3(0.72),
    HIGH_ALGAE(1.035),
    L4(1.32),
    BOTTOM(0);

    private double height;

    private ElevatorHeight(double heightMeters) {
      this.height = heightMeters;
    }

    public double getHeight() {
      return height;
    }
  }

  public final class FieldConstants {
    public static final Translation2d TEAM_RED_REEF_CENTER = new Translation2d(13.059, 4.028);
    public static final Translation2d TEAM_BLUE_REEF_CENTER = new Translation2d(4.489, 4.028);
    public static final Translation2d TEAM_RED_PROCESSOR = new Translation2d(6.026, 0);
    public static final Translation2d TEAM_BLUE_PROCESSOR = new Translation2d(11.585, 8.062);

    public static final Pose2d SOURCE_1 = new Pose2d(16.994, 0.355, Rotation2d.fromDegrees(-45));
    public static final Pose2d SOURCE_2 = new Pose2d(16.725, 7.518, Rotation2d.fromDegrees(45));
    public static final Pose2d SOURCE_12 = new Pose2d(0.648, 0.489, Rotation2d.fromDegrees(30));
    public static final Pose2d SOURCE_13 = new Pose2d(0.702, 7.545, Rotation2d.fromDegrees(135));

    // Field width and length
    public static final double FIELD_X_DIMENSION = 17.548; // Length
    public static final double FIELD_Y_DIMENSION = 8.052; // Width

    public static Pose2d closestSourceTo(Pose2d pose) {
      Pose2d closest = SOURCE_1;
      double closestDistance = pose.getTranslation().getDistance(SOURCE_1.getTranslation());
      double distance = pose.getTranslation().getDistance(SOURCE_2.getTranslation());
      if (distance < closestDistance) {
        closest = SOURCE_2;
        closestDistance = distance;
      }
      distance = pose.getTranslation().getDistance(SOURCE_12.getTranslation());
      if (distance < closestDistance) {
        closest = SOURCE_12;
        closestDistance = distance;
      }
      distance = pose.getTranslation().getDistance(SOURCE_13.getTranslation());
      if (distance < closestDistance) {
        closest = SOURCE_13;
      }
      return closest;
    }
  }

  public enum ReefScoringPose {
    BA1L(0, 0, 0),
    BA1H(0, 0, 1),
    BA2(0, 0, 2),
    BA3(0, 0, 3),
    BA4(0, 0, 4),
    BB1L(0, 1, 0),
    BB1H(0, 1, 1),
    BB2(0, 1, 2),
    BB3(0, 1, 3),
    BB4(0, 1, 4),
    BC1L(1, 0, 0),
    BC1H(1, 0, 1),
    BC2(1, 0, 2),
    BC3(1, 0, 3),
    BC4(1, 0, 4),
    BD1L(1, 1, 0),
    BD1H(1, 1, 1),
    BD2(1, 1, 2),
    BD3(1, 1, 3),
    BD4(1, 1, 4),
    BE1L(2, 0, 0),
    BE1H(2, 0, 1),
    BE2(2, 0, 2),
    BE3(2, 0, 3),
    BE4(2, 0, 4),
    BF1L(2, 1, 0),
    BF1H(2, 1, 1),
    BF2(2, 1, 2),
    BF3(2, 1, 3),
    BF4(2, 1, 4),
    BG1L(3, 0, 0),
    BG1H(3, 0, 1),
    BG2(3, 0, 2),
    BG3(3, 0, 3),
    BG4(3, 0, 4),
    BH1L(3, 1, 0),
    BH1H(3, 1, 1),
    BH2(3, 1, 2),
    BH3(3, 1, 3),
    BH4(3, 1, 4),
    BI1L(4, 0, 0),
    BI1H(4, 0, 1),
    BI2(4, 0, 2),
    BI3(4, 0, 3),
    BI4(4, 0, 4),
    BJ1L(4, 1, 0),
    BJ1H(4, 1, 1),
    BJ2(4, 1, 2),
    BJ3(4, 1, 3),
    BJ4(4, 1, 4),
    BK1L(5, 0, 0),
    BK1H(5, 0, 1),
    BK2(5, 0, 2),
    BK3(5, 0, 3),
    BK4(5, 0, 4),
    BL1L(5, 1, 0),
    BL1H(5, 1, 1),
    BL2(5, 1, 2),
    BL3(5, 1, 3),
    BL4(5, 1, 4),
    RA1L(6, 0, 0),
    RA1H(6, 0, 1),
    RA2(6, 0, 2),
    RA3(6, 0, 3),
    RA4(6, 0, 4),
    RB1L(6, 1, 0),
    RB1H(6, 1, 1),
    RB2(6, 1, 2),
    RB3(6, 1, 3),
    RB4(6, 1, 4),
    RC1L(7, 0, 0),
    RC1H(7, 0, 1),
    RC2(7, 0, 2),
    RC3(7, 0, 3),
    RC4(7, 0, 4),
    RD1L(7, 1, 0),
    RD1H(7, 1, 1),
    RD2(7, 1, 2),
    RD3(7, 1, 3),
    RD4(7, 1, 4),
    RE1L(8, 0, 0),
    RE1H(8, 0, 1),
    RE2(8, 0, 2),
    RE3(8, 0, 3),
    RE4(8, 0, 4),
    RF1L(8, 1, 0),
    RF1H(8, 1, 1),
    RF2(8, 1, 2),
    RF3(8, 1, 3),
    RF4(8, 1, 4),
    RG1L(9, 0, 0),
    RG1H(9, 0, 1),
    RG2(9, 0, 2),
    RG3(9, 0, 3),
    RG4(9, 0, 4),
    RH1L(9, 1, 0),
    RH1H(9, 1, 1),
    RH2(9, 1, 2),
    RH3(9, 1, 3),
    RH4(9, 1, 4),
    RI1L(10, 0, 0),
    RI1H(10, 0, 1),
    RI2(10, 0, 2),
    RI3(10, 0, 3),
    RI4(10, 0, 4),
    RJ1L(10, 1, 0),
    RJ1H(10, 1, 1),
    RJ2(10, 1, 2),
    RJ3(10, 1, 3),
    RJ4(10, 1, 4),
    RK1L(11, 0, 0),
    RK1H(11, 0, 1),
    RK2(11, 0, 2),
    RK3(11, 0, 3),
    RK4(11, 0, 4),
    RL1L(11, 1, 0),
    RL1H(11, 1, 1),
    RL2(11, 1, 2),
    RL3(11, 1, 3),
    RL4(11, 1, 4);

    private final int side;
    private final int index;
    private final int height;

    private ReefScoringPose(int side, int index, int height) {
      this.side = side;
      this.index = index;
      this.height = height;
    }

    public static ReefScoringPose closestTo(Pose3d pose, double maxDistance) {
      ReefScoringPose closest = null;
      double closestDistance = Double.MAX_VALUE;
      for (ReefScoringPose reef : ReefScoringPose.values()) {
        double distance = reef.getPose().getTranslation().getDistance(pose.getTranslation());
        if (distance <= maxDistance && distance < closestDistance) {
          closest = reef;
          closestDistance = distance;
        }
      }
      return closest;
    }

    private static final double SIDE_LENGTH = 0.16;
    private static final double RADIUS = 0.7;

    public Pose3d getPose() {
      double sideRelative = side % 6;
      double theta = -sideRelative * Math.PI / 3;
      double theta2 = theta + Math.PI / 2;

      double cx;
      double cy;

      if (side < 6) {
        cx = FieldConstants.TEAM_BLUE_REEF_CENTER.getX();
        cy = FieldConstants.TEAM_BLUE_REEF_CENTER.getY();
      } else {
        cx = FieldConstants.TEAM_RED_REEF_CENTER.getX();
        cy = FieldConstants.TEAM_RED_REEF_CENTER.getY();
      }

      double radius = RADIUS;
      if (height == 4) {
        radius += 0.08;
      } else if (height == 0) {
        radius += 0.066;
      } else if (height == 1) {
        radius -= 0.04;
      }

      double x = cx - radius * Math.cos(theta);
      double y = cy + radius * Math.sin(theta);

      double len = SIDE_LENGTH;

      if (index == 1) len = -len;

      double rx = x + len * Math.cos(theta2);
      double ry = y - len * Math.sin(theta2);

      double height;
      double yaw = -theta;
      double pitch = 32;
      switch (this.height) {
        case 0:
          height = 0.485;
          pitch = 0;
          yaw = -theta2;
          break;
        case 1:
          height = 0.52;
          pitch = 0;
          yaw = -theta2;
          break;
        case 2:
          height = 0.71;
          break;
        case 3:
          height = 1.11;
          break;
        case 4:
          height = 1.75;
          pitch = 90;
          break;
        default:
          height = 0.0;
          break;
      }

      return new Pose3d(
          new Translation3d(rx, ry, height), new Rotation3d(0, Units.degreesToRadians(pitch), yaw));
    }
  }

  public enum FieldPosition {
    ReefCenter(
        Constants.FieldConstants.TEAM_RED_REEF_CENTER,
        Constants.FieldConstants.TEAM_BLUE_REEF_CENTER),

    Processor(
        Constants.FieldConstants.TEAM_RED_PROCESSOR, Constants.FieldConstants.TEAM_BLUE_PROCESSOR);

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
        new Pose2d(1.235, 4.001, new Rotation2d(0))); // TODO is old?

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
    public static final double kMaxVelocity = 8;
    public static final double kMaxAcceleration = 10;
    public static final double kV = 5.4352;
    public static final double kA = 0.088399;
    public static final double kG = 0.088227;
    public static final double kS = 0.09213;

    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(10);
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);

    public static final double STD_STATE_POSITION = 3.0; // m
    public static final double STD_STATE_VELOCITY = 3.0; // m/s
    public static final double STD_ENCODER_POSITION = 0.001; // m
    public static final double STD_ENCODER_VELOCITY = 0.001; // m/s

    public static final double REGULATOR_POSITION_ERROR_TOLERANCE =
        0.03; // (m) tolerance for error, decrease to make regulator more aggressive
    public static final double REGULATOR_VELOCITY_ERROR_TOLERANCE =
        0.63514; // (m/s) tolerance for error, decrease to make regulator more aggressive

    public static final double REGULATOR_CONTROL_EFFORT_TOLERANCE =
        12.0; // (V) max control effort, decrease to make regulator more lazy

    public static final double GEAR_RATIO = 13.4321 / 2.0 /* compensation for second stage ratio */;
    public static final double DRUM_RADIUS = Units.inchesToMeters(0.8783343);
    // 2 * pi * r / gear ratio because same as getting distance a wheel moved, just vertically
    public static final double METERS_PER_ROTATION = DRUM_RADIUS * 2 * Math.PI / GEAR_RATIO;

    public static final double AT_GOAL_POSITION_TOLERANCE = REGULATOR_POSITION_ERROR_TOLERANCE;
    public static final double AT_GOAL_VELOCITY_TOLERANCE = REGULATOR_VELOCITY_ERROR_TOLERANCE;

    public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0.15, 0, Rotation2d.fromDegrees(0));
    public static final double MIN_LENGTH = 0.65;
    public static final double MAX_HEIGHT = 1.443284;
  }

  public final class CoralShooter {
    public static final double kP = 0.07;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.01;
    public static final double kV = 0.1;
    public static final double kA = 0.04;

    public static final double OUT_SPEED_TOP = 4; // TODO this is probably too slow
    public static final double OUT_SPEED_BOTTOM = 4; // TODO this is probably too slow
    public static final double INTAKE_SPEED_TOP = -4; // TODO ^^^^^^^^^^^
    public static final double INTAKE_SPEED_BOTTOM = -4; // TODO ^^^^^^^^^^^

    public static final double SHOOT_TIME = 0.1; // TODO make correct

    public static final double LENGTH = 0.25;
  }

  public final class CoralIntake {

    public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(10);
    public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(60);

    // TODO: need correct values
    public static final double kP = 0.07;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.12;
    public static final double kV = 0.14;
    public static final double kA = 0.03;

    public static final double MAX_ARM_VELOCITY = 8;
    public static final double MAX_ARM_ACCELERATION = 22;

    public static final double REVERSE_SPEED = 4; // TODO this is probably too slow
    public static final double INTAKE_SPEED = -4; // TODO ^^^^^^^^^^^

    public static final double INTAKE_TIME = 0.1; // TODO make correct

    public static final double sP = 7.6661;
    public static final double sI = 0;
    public static final double sD = 0.80357;
    public static final double sG = 0.33478;
    public static final double sS = 0.0094404;
    public static final double sV = 0.98364;
    public static final double sA = 0.086232;

    public static final double ARM_UP = Units.degreesToRadians(80);
    public static final double ARM_DOWN = -1.1;
    public static final double ARM_START_POS = Units.degreesToRadians(-25); // Math.PI / 2;

    public static final double ARM_GEAR_RATIO = 56.8889;

    public static final double DEBOUNCE_TIME = 0.05; // TODO make correct

    public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0, 0.4, Rotation2d.fromDegrees(0));
    public static final double LENGTH = 0.431;
    public static final double ANGLE_OFFSET = 0;
  }

  public final class Lights {
    public static final int length = 150;

    public static final Dimensionless multiplier = Percent.of(100);

    public static final Time pulsateFrequency = Seconds.of(1);

    public static final double FLASH_GREEN_TIME = 1; // seconds

    public static enum LightSegments {
      ELEVATOR,
      GROUND_ALGAE,
      CORAL_INTAKE,
      CORAL_SHOOTER,
      HYBRID_STATES
    }

    public static final Map<LightSegments, Pair<Integer, Integer>> PART_INDECIES =
        Map.of(
            LightSegments.ELEVATOR, Pair.of(0, 5),
            LightSegments.GROUND_ALGAE, Pair.of(6, 10),
            LightSegments.CORAL_INTAKE, Pair.of(11, 15),
            LightSegments.CORAL_SHOOTER, Pair.of(16, 20),
            LightSegments.HYBRID_STATES, Pair.of(21, 25));

    public static final LEDPattern PULSATING_ORANGE =
        LEDPattern.solid(Color.kOrange)
            .breathe(Constants.Lights.pulsateFrequency)
            .blend(LEDPattern.solid(Color.kOrange));
    public static final LEDPattern PULSATING_GREEN =
        LEDPattern.solid(Color.kGreen).breathe(Constants.Lights.pulsateFrequency);
    public static final LEDPattern OFF = LEDPattern.solid(Color.kBlack);

    public static final LEDPattern elevatorPattern =
        PULSATING_GREEN
            .mask(
                LEDPattern.progressMaskLayer(
                    () ->
                        RobotContainer.elevator.getCurrentHeight() / Constants.Elevator.MAX_HEIGHT))
            .overlayOn(PULSATING_ORANGE);
    public static final Supplier<LEDPattern> coralIntakePattern =
        () ->
            LEDPattern.solid(
                Color.lerpRGB(
                    Color.kRed,
                    Color.kGreen,
                    (RobotContainer.coralIntake.getArmAngle() - Constants.CoralIntake.ARM_UP)
                        / (Constants.CoralIntake.ARM_DOWN
                            - Constants.CoralIntake
                                .ARM_UP))); // TODO also bad (but not the worst ive written today)
    public static final Supplier<LEDPattern> coralShooterPattern =
        () ->
            (RobotContainer.coralShooter.getCurrentState() == CoralShooterStates.OFF)
                ? PULSATING_GREEN
                : PULSATING_ORANGE;

    public static final LEDPattern sourcePattern = LEDPattern.solid(Color.kRed);
    public static final LEDPattern reefScorePattern = LEDPattern.solid(Color.kBlue);
    public static final LEDPattern algaeScorePattern = LEDPattern.solid(Color.kPurple);
    public static final LEDPattern cagePattern = LEDPattern.solid(Color.kOrange);
    public static final LEDPattern removeAlgaePattern = LEDPattern.solid(Color.kYellowGreen);
    public static final LEDPattern coralScorePattern = LEDPattern.solid(Color.kAqua);

    public static final Map<LightSegments, Supplier<LEDPattern>> DEFAULT_PATTERNS =
        Map.of(
            LightSegments.ELEVATOR, () -> OFF,
            LightSegments.GROUND_ALGAE, () -> OFF,
            LightSegments.CORAL_INTAKE, () -> OFF,
            LightSegments.CORAL_SHOOTER, () -> OFF);
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

    public static final double BUMPER_WIDTH = 0.762;
    public static final double MAX_MECHANISM_HEIGHT = 2.1336;
  }

  public final class Swerve {

    public static double kDt = 0.020; // 0.003;

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

    public static final Current FALCON_TURN_STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current FALCON_TURN_SUPPLY_CURRENT_LIMIT = Amps.of(25);
    public static final Current FALCON_DRIVE_STATOR_CURRENT_LIMIT = Amps.of(120);
    public static final Current FALCON_DRIVE_SUPPLY_CURRENT_LIMIT = Amps.of(32);

    public static final Current KRAKEN_TURN_STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current KRAKEN_TURN_SUPPLY_CURRENT_LIMIT = Amps.of(25);
    public static final Current KRAKEN_DRIVE_STATOR_CURRENT_LIMIT = Amps.of(120);
    public static final Current KRAKEN_DRIVE_SUPPLY_CURRENT_LIMIT = Amps.of(32);

    public static final double FALCON_DRIVE_KS = 0.12373;
    public static final double FALCON_DRIVE_KV = 2.5609;
    public static final double FALCON_DRIVE_KA = 0.10075;
    public static final double FALCON_DRIVE_STD_STATE_VELOCITY = 3;
    public static final double FALCON_DRIVE_STD_ENCODER_VELOCITY = 0.01;
    public static final double FALCON_DRIVE_REGULATOR_VELOCITY_ERROR_TOLERANCE = 1.6999;
    public static final double FALCON_DRIVE_REGULATOR_CONTROL_EFFORT_TOLERANCE = 7.0;

    public static final double FALCON_TURN_KV = 1.7518;
    public static final double FALCON_TURN_KA = 0.015791;
    public static final double FALCON_TURN_KS = 0.08889;
    public static final double FALCON_TURN_STD_STATE_POSITION = 3;
    public static final double FALCON_TURN_STD_STATE_VELOCITY = 3;
    public static final double FALCON_TURN_STD_ENCODER_POSITION = 0.01;
    public static final double FALCON_TURN_STD_ENCODER_VELOCITY = 0.01;
    public static final double FALCON_TURN_REGULATOR_POSITION_ERROR_TOLERANCE = 0.02;
    public static final double FALCON_TURN_REGULATOR_VELOCITY_ERROR_TOLERANCE = 1.6999;
    public static final double FALCON_TURN_REGULATOR_CONTROL_EFFORT_TOLERANCE = 7.0;

    public static final double KRAKEN_TURN_KV = 1.7519;
    public static final double KRAKEN_TURN_KA = 0.017189;
    public static final double KRAKEN_TURN_KS = 0.001;
    public static final double KRAKEN_TURN_STD_STATE_POSITION = 2;
    public static final double KRAKEN_TURN_STD_STATE_VELOCITY = 2;
    public static final double KRAKEN_TURN_STD_ENCODER_POSITION = 0.1;
    public static final double KRAKEN_TURN_STD_ENCODER_VELOCITY = 0.1;
    public static final double KRAKEN_TURN_REGULATOR_POSITION_ERROR_TOLERANCE = 0.1;
    public static final double KRAKEN_TURN_REGULATOR_VELOCITY_ERROR_TOLERANCE = 1.693;
    public static final double KRAKEN_TURN_REGULATOR_CONTROL_EFFORT_TOLERANCE = 7.0;

    public static final double KRAKEN_TURN_KP = 2.3;
    public static final double KRAKEN_TURN_KI = 0;
    public static final double KRAKEN_TURN_KD = 0;

    public static final double KRAKEN_DRIVE_KS = 0.12373;
    public static final double KRAKEN_DRIVE_KV = 2.5609;
    public static final double KRAKEN_DRIVE_KA = 0.10075;
    public static final double KRAKEN_DRIVE_STD_STATE_VELOCITY = 3;
    public static final double KRAKEN_DRIVE_STD_ENCODER_VELOCITY = 0.01;
    public static final double KRAKEN_DRIVE_REGULATOR_VELOCITY_ERROR_TOLERANCE = 1.6999;
    public static final double KRAKEN_DRIVE_REGULATOR_CONTROL_EFFORT_TOLERANCE = 7.0;

    // Wheel diameter
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

    // Turn max velocity and acceleration
    // Calculated from motor rpm 5000 / 60 (rps) / gear ratio (15.43)
    public static final double TurnMaxAngularVelocity = 25; // ROTATIONS / SECOND
    // Calculated from max velocity / time to reach (0.1)
    public static final double TurnMaxAngularAcceleration = 15; // ROTATIONS / SECOND / SECOND

    /** The max speed the robot can travel safely */
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
      return RobotBase.isReal() ? Mode.REAL : RobotBase.isSimulation() ? Mode.SIM : Mode.REPLAY;
    }

    public static final boolean MOTOR_LOGGING_ENABLED = true;

    public static enum Mode {
      REAL,
      SIM,
      REPLAY
    }
  }
}
