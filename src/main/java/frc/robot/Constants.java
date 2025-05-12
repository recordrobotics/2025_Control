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
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.MotorLocation;
import frc.robot.utils.ModuleConstants.MotorType;
import java.util.HashMap;
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
  public final class Align {
    public static final double MAX_VELOCITY = 1.5; // m/s
    public static final double MAX_ACCELERATION = 3.0; // m/s^2
    public static final double MAX_ANGULAR_VELOCITY = 4.0; // rad/s
    public static final double MAX_ANGULAR_ACCELERATION = 7.0; // rad/s^2

    public static final double translationalTolerance = 0.01; // Meters
    public static final double rotationalTolerance = 0.02; // Radians
  }

  public final class PhotonVision {
    public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final String PHOTON_L1_NAME = "photon-l1";
    public static final String PHOTON_SOURCE_NAME = "photon-source";
    public static final String PHOTON_CORAL_INTAKE = "coral-intake";

    public static final Transform3d l1TransformRobotToCamera =
        new Transform3d(
            new Translation3d(-0.122023, -0.372571, 0.197230),
            new Rotation3d(0, Units.degreesToRadians(-12.894), Units.degreesToRadians(90)));
    public static final Transform3d sourceTransformRobotToCamera =
        new Transform3d(
            new Translation3d(0.019276, 0.370521, 0.197763),
            new Rotation3d(0, Units.degreesToRadians(-18.951), Units.degreesToRadians(-90)));
    public static final Transform3d groundIntakeTransformRobotToCamera =
        new Transform3d(
            new Translation3d(Meters.of(0.2), Meters.of(-0.372571), Inches.of(31)),
            new Rotation3d(
                Degrees.of(0), Degrees.of(-22), Degrees.of(90))); // TODO: change to cad value

    public static final Distance CORAL_ID_DISTANCE = Inches.of(8);
    public static final Time CORAL_TIMEOUT = Seconds.of(0.5);

    public static final int CORAL_ID = 1;

    public static final double ROT_STD_DEV_WHEN_TRUSTING = 4;
  }

  public final class HybridConstants {
    // Create the constraints to use while pathfinding. The constraints defined in the path will
    // only be used for the path. docs are here:
    // http://gabybot.com/RobotCoreDoc/classcom_1_1pathplanner_1_1lib_1_1path_1_1_path_constraints.html
    public static final PathConstraints constraints =
        new PathConstraints(
            3, // Max velocity meters per second
            9, // Max acceleration meters per second per second
            Units.degreesToRadians(540), // Max angular velocity radians per second
            Units.degreesToRadians(720)); // Max angular acceleration radians per second per second

    public static final double processorTriggerDistance = 3.0; // Meters
    public static final double reefTriggerDistance = 3.0; // Meters

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
    public static final String LIMELIGHT_LEFT_NAME = "limelight-left";
    public static final String LIMELIGHT_CENTER_NAME = "limelight-center";

    public static final Transform3d leftTransformRobotToCamera =
        new Transform3d(
            new Translation3d(0.311558, -0.330204, 0.246383),
            new Rotation3d(0, Units.degreesToRadians(-21), 0));
    public static final Transform3d centerTransformRobotToCamera =
        new Transform3d(
            new Translation3d(0.219412, 0.050800, 0.156247),
            new Rotation3d(0, Units.degreesToRadians(-27), 0));

    public static final double ROT_STD_DEV_WHEN_TRUSTING = 4;
  }

  public final class Assits {
    public static final Distance GROUND_ASSIST_MAX_CORAL_DISTANCE = Meters.of(2);
    public static final Angle GROUND_ASSIST_MAX_ANGLE_ERROR = Degrees.of(60);
    public static final double GROUND_ASSIST_ROTATION_P = 5.0;
    public static final double GROUND_ASSIST_TRANSLATION_P = 9.0;
  }

  public final class Climber {
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);

    public static final double MAX_ARM_VELOCITY = 1;
    public static final double MAX_ARM_ACCELERATION = 1;

    public static final double kP = 2.996; // TODO: tune sysid
    public static final double kI = 0;
    public static final double kD = 0.28181;
    public static final double kG = 0.2269;
    public static final double kS = 0.13495;
    public static final double kV = 0.67446;
    public static final double kA = 0.018492;

    public static final Angle START_ROTATIONS = Rotations.of(0);
    public static final Angle PARK_ROTATIONS = START_ROTATIONS;
    public static final Angle EXTENDED_ROTATIONS = Rotations.of(2); // TODO: make correct
    public static final Angle CLIMBED_ROTATIONS = Rotations.of(1); // TODO: make correct

    public static final double GEAR_RATIO = 48; // 48:1 gearbox

    // TODO: make correct
    public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0, 0.4, Rotation2d.fromDegrees(0));
    // TODO: make correct
    public static final Distance LENGTH = Meters.of(0.431);
    public static final double ANGLE_OFFSET = 0;
  }

  public final class ElevatorArm {
    public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(10);
    public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(60);

    public static final double MAX_ARM_VELOCITY = 12;
    public static final double MAX_ARM_ACCELERATION = 40;

    public static final double kP = 2.996;
    public static final double kI = 0;
    public static final double kD = 0.28181;
    public static final double kG = 0.53175;
    public static final double kS = 0.17279;
    public static final double kV = 0.53109;
    public static final double kA = 0.048931;

    public static final double START_POS = Units.degreesToRadians(-90);
    public static final double MIN_POS = Units.degreesToRadians(-103.54);
    public static final double MAX_POS = Units.degreesToRadians(113.79);

    public static final double ARM_GEAR_RATIO = 38.4; // 16:1 * 72/30

    // TODO: make correct
    public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0, 0.4, Rotation2d.fromDegrees(0));
    // TODO: make correct
    public static final double LENGTH = 0.431;
    public static final double ANGLE_OFFSET = 0;

    public static final Distance MANUAL_CONTROL_MARGIN = Meters.of(0.1);
  }

  public enum ElevatorHeight {
    INTAKE(Units.inchesToMeters(0.1), Units.degreesToRadians(-103.54)),
    L1(0.26, Units.degreesToRadians(-101.69)),
    L2(0.458, Units.degreesToRadians(-101.69)),
    LOW_REEF_ALGAE(0.4849, Units.degreesToRadians(-37.841)),
    L3(0.868, Units.degreesToRadians(-101.59)),
    HIGH_REEF_ALGAE(0.916, Units.degreesToRadians(-40.26)),
    // L4(1.299, Units.degreesToRadians(64.85)),
    L4(1.309, Units.degreesToRadians(54.85)),
    BOTTOM(Constants.Elevator.LOWEST_HOLD_HEIGHT, Units.degreesToRadians(-90)),
    GROUND_ALGAE(Constants.Elevator.LOWEST_HOLD_HEIGHT, Units.degreesToRadians(-36)),
    GROUND_ALGAE_PROCESSOR(0.04, Units.degreesToRadians(5)),
    PROCESSOR_SCORE(0.04, Units.degreesToRadians(-60)),
    BARGE_ALAGAE(1.309, Units.degreesToRadians(110));

    private double height;
    private double armAngleRadians;

    private ElevatorHeight(double heightMeters, double armAngleRadians) {
      this.height = heightMeters;
      this.armAngleRadians = armAngleRadians;
    }

    public double getHeight() {
      return height;
    }

    public double getArmAngle() {
      return armAngleRadians;
    }

    public double getDifference(double height, double angle) {
      return Math.abs(this.height - height) / 0.1
          + Math.abs(this.armAngleRadians - angle) / Units.degreesToRadians(4);
    }
  }

  public final class FieldConstants {
    public static final Translation2d TEAM_BLUE_REEF_CENTER = new Translation2d(4.489, 4.028);
    public static final Translation2d TEAM_RED_REEF_CENTER = new Translation2d(13.059, 4.028);
    public static final Translation2d TEAM_BLUE_PROCESSOR = new Translation2d(6.026, 0);
    public static final Translation2d TEAM_RED_PROCESSOR = new Translation2d(11.585, 8.062);

    public static final Pose2d SOURCE_RL =
        new Pose2d(16.994, 0.355, Rotation2d.fromDegrees(35.988608));
    public static final Pose2d SOURCE_RR =
        new Pose2d(16.725, 7.518, Rotation2d.fromDegrees(-215.988608));
    public static final Pose2d SOURCE_BR =
        new Pose2d(0.648, 0.489, Rotation2d.fromDegrees(-35.988608));
    public static final Pose2d SOURCE_BL =
        new Pose2d(0.702, 7.545, Rotation2d.fromDegrees(215.988608));

    // Field width and length
    public static final double FIELD_X_DIMENSION = 17.548; // Length
    public static final double FIELD_Y_DIMENSION = 8.052; // Width

    public static Pose2d closestSourceTo(Pose2d pose) {
      Pose2d closest = SOURCE_RL;
      double closestDistance = pose.getTranslation().getDistance(SOURCE_RL.getTranslation());
      double distance = pose.getTranslation().getDistance(SOURCE_RR.getTranslation());
      if (distance < closestDistance) {
        closest = SOURCE_RR;
        closestDistance = distance;
      }
      distance = pose.getTranslation().getDistance(SOURCE_BR.getTranslation());
      if (distance < closestDistance) {
        closest = SOURCE_BR;
        closestDistance = distance;
      }
      distance = pose.getTranslation().getDistance(SOURCE_BL.getTranslation());
      if (distance < closestDistance) {
        closest = SOURCE_BL;
      }
      return closest;
    }
  }

  public enum RobotAlignPose {
    // Reefs
    BA(new Pose2d(2.4993371000000004, 4.0181723, Rotation2d.fromDegrees(0)), true),
    RA(new Pose2d(15.0489133, 4.033729300000001, Rotation2d.fromDegrees(180)), true),
    BB(new Pose2d(2.4993371000000004, 3.6695533000000005, Rotation2d.fromDegrees(0)), true),
    RB(new Pose2d(15.0489133, 4.3823483, Rotation2d.fromDegrees(180)), true),
    BC(new Pose2d(3.5010734786033373, 2.2986709964689673, Rotation2d.fromDegrees(60)), true),
    RC(new Pose2d(14.047176921396662, 5.753230603531033, Rotation2d.fromDegrees(-120)), true),
    BD(new Pose2d(3.802986388845264, 2.1243614964689677, Rotation2d.fromDegrees(60)), true),
    RD(new Pose2d(13.745264011154736, 5.927540103531033, Rotation2d.fromDegrees(-120)), true),
    BE(new Pose2d(5.491073478603337, 2.306449496468968, Rotation2d.fromDegrees(120)), true),
    RE(new Pose2d(12.057176921396664, 5.745452103531033, Rotation2d.fromDegrees(-60)), true),
    BF(new Pose2d(5.792986388845264, 2.4807589964689676, Rotation2d.fromDegrees(120)), true),
    RF(new Pose2d(11.755264011154736, 5.571142603531033, Rotation2d.fromDegrees(-60)), true),
    BG(new Pose2d(6.4793371, 4.0337293, Rotation2d.fromDegrees(180)), true),
    RG(new Pose2d(11.0689133, 4.018172300000001, Rotation2d.fromDegrees(0)), true),
    BH(new Pose2d(6.4793371, 4.3823483, Rotation2d.fromDegrees(180)), true),
    RH(new Pose2d(11.0689133, 3.6695533000000005, Rotation2d.fromDegrees(0)), true),
    BI(new Pose2d(5.477600721396662, 5.753230603531033, Rotation2d.fromDegrees(-120)), true),
    RI(new Pose2d(12.070649678603338, 2.2986709964689673, Rotation2d.fromDegrees(60)), true),
    BJ(new Pose2d(5.175687811154734, 5.927540103531033, Rotation2d.fromDegrees(-120)), true),
    RJ(new Pose2d(12.372562588845266, 2.1243614964689677, Rotation2d.fromDegrees(60)), true),
    BK(new Pose2d(3.487600721396663, 5.745452103531033, Rotation2d.fromDegrees(-60)), true),
    RK(new Pose2d(14.060649678603337, 2.306449496468968, Rotation2d.fromDegrees(120)), true),
    BL(new Pose2d(3.185687811154736, 5.571142603531033, Rotation2d.fromDegrees(-60)), true),
    RL(new Pose2d(14.362562588845265, 2.4807589964689676, Rotation2d.fromDegrees(120)), true),
    // Processors
    BProcessor(new Pose2d(6, 0.43, Rotation2d.fromDegrees(270)), true),
    RProcessor(new Pose2d(11.5, 7.59, Rotation2d.fromDegrees(90)), true),

    BSourceOuterLeft(new Pose2d(1.367, 7.274, Rotation2d.fromDegrees(35.734)), false),
    BSourceOuterRight(new Pose2d(1.211, 0.872, Rotation2d.fromDegrees(144.118)), false),
    RSourceOuterLeft(
        new Pose2d(FlippingUtil.fieldSizeX - 1.211, 0.872, Rotation2d.fromDegrees(-144.118)),
        false),
    RSourceOuterRight(
        new Pose2d(FlippingUtil.fieldSizeX - 1.367, 7.274, Rotation2d.fromDegrees(-35.734)), false);
    // BSourceInnerLeft(new Pose2d(0.683, 6.711, Rotation2d.fromDegrees(36.209)), false),
    // BSourceInnerRight(new Pose2d(0.693, 1.315, Rotation2d.fromDegrees(143.596)), false),
    // RSourceInnerLeft(
    //     new Pose2d(FlippingUtil.fieldSizeX - 0.693, 1.315, Rotation2d.fromDegrees(-143.791)),
    //     false),
    // RSourceInnerRight(
    //     new Pose2d(FlippingUtil.fieldSizeX - 0.683, 6.711, Rotation2d.fromDegrees(-36.209)),
    // false);

    private Pose2d pose;
    private boolean useTranslation;

    private RobotAlignPose(Pose2d pose, boolean useTranslation) {
      this.pose = pose;
      this.useTranslation = useTranslation;
    }

    public Pose2d getFirstStagePose() {
      return pose.transformBy(new Transform2d(new Translation2d(-0.2 + 0.6, 0), Rotation2d.kZero));
    }

    public Pose2d getNearPose() {
      return pose.transformBy(new Transform2d(new Translation2d(0.1 + 0.6, 0), Rotation2d.kZero));
    }

    public Pose2d getFarPose() {
      return pose.transformBy(new Transform2d(new Translation2d(0.05 + 0.6, 0), Rotation2d.kZero));
    }

    public boolean useTranslation() {
      return useTranslation;
    }

    public static RobotAlignPose closestTo(Pose2d pose, double maxDistance) {
      RobotAlignPose closest = null;
      double closestDistance = Double.MAX_VALUE;
      for (RobotAlignPose align : RobotAlignPose.values()) {
        double distance = align.getFarPose().getTranslation().getDistance(pose.getTranslation());
        if (distance <= maxDistance && distance < closestDistance) {
          closest = align;
          closestDistance = distance;
        }
      }
      return closest;
    }

    public static final RobotAlignPose[] leftReefPoses = {
      RobotAlignPose.BA,
      RobotAlignPose.BC,
      RobotAlignPose.BF,
      RobotAlignPose.BH,
      RobotAlignPose.BJ,
      RobotAlignPose.BK,
      RobotAlignPose.RA,
      RobotAlignPose.RC,
      RobotAlignPose.RF,
      RobotAlignPose.RH,
      RobotAlignPose.RJ,
      RobotAlignPose.RK
    };

    public static final RobotAlignPose[] rightReefPoses = {
      RobotAlignPose.BB,
      RobotAlignPose.BD,
      RobotAlignPose.BE,
      RobotAlignPose.BG,
      RobotAlignPose.BI,
      RobotAlignPose.BL,
      RobotAlignPose.RB,
      RobotAlignPose.RD,
      RobotAlignPose.RE,
      RobotAlignPose.RG,
      RobotAlignPose.RI,
      RobotAlignPose.RL
    };

    public static final RobotAlignPose[] reefPoses = {
      RobotAlignPose.BA,
      RobotAlignPose.BB,
      RobotAlignPose.BC,
      RobotAlignPose.BD,
      RobotAlignPose.BE,
      RobotAlignPose.BF,
      RobotAlignPose.BG,
      RobotAlignPose.BH,
      RobotAlignPose.BI,
      RobotAlignPose.BJ,
      RobotAlignPose.BL,
      RobotAlignPose.BK,
      RobotAlignPose.RA,
      RobotAlignPose.RB,
      RobotAlignPose.RC,
      RobotAlignPose.RD,
      RobotAlignPose.RE,
      RobotAlignPose.RF,
      RobotAlignPose.RG,
      RobotAlignPose.RH,
      RobotAlignPose.RI,
      RobotAlignPose.RJ,
      RobotAlignPose.RL,
      RobotAlignPose.RK
    };

    public static RobotAlignPose closestReefTo(Pose2d pose, double maxDistance) {
      RobotAlignPose closest = null;
      double closestDistance = Double.MAX_VALUE;
      for (RobotAlignPose align : reefPoses) {
        double distance = align.getFarPose().getTranslation().getDistance(pose.getTranslation());
        if (distance <= maxDistance && distance < closestDistance) {
          closest = align;
          closestDistance = distance;
        }
      }
      return closest;
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
    BargeLeft(new Pose2d(7.004, 6.171, Rotation2d.fromDegrees(-152.840))),
    BargeCenter(new Pose2d(7.145, 4.026, Rotation2d.fromDegrees(180.000))),
    BargeRight(new Pose2d(6.961, 1.939, Rotation2d.fromDegrees(127.847))),
    ReefTest(new Pose2d(2.082, 4.053, Rotation2d.fromDegrees(-120))),
    ProccessorTest(new Pose2d(6.148, 0.606, Rotation2d.fromDegrees(-90)));

    private final Pose2d m_transformRed;
    private final Pose2d m_transformBlue;

    private FieldStartingLocation(Pose2d poseBlue) {
      m_transformRed = FlippingUtil.flipFieldPose(poseBlue);
      m_transformBlue = poseBlue;
    }

    public Pose2d getPose() {
      return DriverStationUtils.getCurrentAlliance() == Alliance.Red
          ? m_transformRed
          : m_transformBlue;
    }
  }

  public final class Elevator {
    public static final double kDt = 0.02;
    public static final double kMaxVelocity = 2.3;
    public static final double kMaxAcceleration = 10;
    public static final double kV = 5.5255;
    public static final double kA = 0.1319;
    public static final double kG = 0.19133;
    public static final double kS = 0.078513;

    public static final double STARTING_HEIGHT = 0;
    public static final double LOWEST_HOLD_HEIGHT = Units.inchesToMeters(1.25);

    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);
    public static final Current SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(30);
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);

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
    public static final double MAX_HEIGHT = 1.339;

    public static final Distance MANUAL_CONTROL_MARGIN = Meters.of(0.1);
  }

  public final class ElevatorHead {
    public static final double kP = 0.047421;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kP_position = 6.9;
    public static final double kI_position = 0.0;
    public static final double kD_position = 0.132;
    public static final double kS = 0.19168;
    public static final double kV = 3.1111;
    public static final double kA = 0.23988;

    public static final double GEAR_RATIO = 10.0;
    public static final Distance DISTANCE_BETWEEN_AXLES = Inches.of(7);
    public static final Distance CORAL_OUTER_DIAMETER = Inches.of(4.5);
    public static final Distance WHEEL_DIAMETER =
        DISTANCE_BETWEEN_AXLES.minus(CORAL_OUTER_DIAMETER);

    public static final Distance CORAL_INTAKE_DISTANCE = Inches.of(1.2);

    // m/s coral
    public static final double POSITION_MODE_MAX_VELOCITY = 1.4;
    public static final double POSITION_MODE_MAX_ACCELERATION = 0.7;

    public static final double OUT_SPEED_FORWARD = 2.5;
    public static final double OUT_SPEED_BACKWARD = -1.5;
    public static final double INTAKE_SPEED = 0.85; // -0.35;

    public static final double AT_GOAL_POSITION_TOLERANCE = 0.05;
    public static final double AT_GOAL_VELOCITY_TOLERANCE = 0.07;

    public static final Time SHOOT_TIME = Seconds.of(0.3);

    public static final double DEBOUNCE_TIME = 0.02;

    public static final double HOW_FAR_FORWARDS_FROM_THE_ELEVATOR_IS_THE_CORAL_SHOOTER = 0.25;

    public static final double OUT_GROUND_SPEED = -8.8;
    public static final double OUT_REEF_SPEED = 2.8;
    public static final double SHOOT_BARGE_SPEED = -16;
    public static final double INTAKE_GROUND_SPEED = 4.2;
    public static final double INTAKE_REEF_SPEED = -5.5;
    public static final double HOLD_GROUND_SPEED = 1.15;
    public static final double HOLD_REEF_SPEED = -5.4;

    public static final double SHOOT_TIME_GROUND = 2.3;
    public static final double SHOOT_TIME_REEF = 0.3;
    public static final double SHOOT_TIME_BARGE = 0.4;
  }

  public final class CoralIntake {

    public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(10);
    public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(60);

    public static final double wheel_kP = 0.037121;
    public static final double wheel_kI = 0.0;
    public static final double wheel_kD = 0.0;
    public static final double wheel_kS = 0.1709;
    public static final double wheel_kV = 0.61705;
    public static final double wheel_kA = 0.039556;

    public static final double MAX_ARM_VELOCITY = 8;
    public static final double MAX_ARM_ACCELERATION = 22;

    public static final double PUSH_OUT_SPEED = 8;
    public static final double PULL_THROUGH_SPEED = 18;
    public static final double SOURCE_SPEED = 18;
    public static final double INTAKE_SPEED = -18;
    public static final double L1_SCORE_SPEED = 20;

    public static final double PUSH_OUT_RAMP_TIME = 0.5;

    public static final double arm_kP = 4.8919;
    public static final double arm_kI = 0;
    public static final double arm_kD = 0.11072;
    public static final double arm_kG = 0.37793;
    public static final double arm_kS = 0.30532;
    public static final double arm_kV = 0.79648;
    public static final double arm_kA = 0.038737;

    public static final double ARM_UP = Units.degreesToRadians(80.19);
    public static final double ARM_PUSH = Units.degreesToRadians(84.19);
    public static final double ARM_INTAKE = Units.degreesToRadians(77.08);
    public static final double ARM_SCORE_L1 = Units.degreesToRadians(22.35);
    public static final double ARM_DOWN = Units.degreesToRadians(-49.5);
    public static final double ARM_START_POS = Units.degreesToRadians(89.3);

    public static final double ARM_GEAR_RATIO = 56.8889; // 16:1 * 64/18

    public static final double WHEEL_GEAR_RATIO = 10;

    public static final double SHOOT_TIME = 0.3;

    public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0, 0.4, Rotation2d.fromDegrees(0));
    public static final double LENGTH = 0.385;
    public static final double ANGLE_OFFSET = 0;
  }

  public final class Lights {
    public static final int LENGTH = 150;

    public static final Dimensionless MULTIPLIER = Percent.of(100);

    public static final Time PULSATE_FREQUENCY = Seconds.of(0.4);
    public static final Time FLASH_FREQUENCY = Seconds.of(0.2);

    public static final double SUCCESS_FLASH_TIME = 1; // seconds

    public static final LinearVelocity SCROLL_SPEED = MetersPerSecond.of(0.3);
    public static final Distance LED_SPACING = Meters.of(1.0 / 30.0); // 30 LEDs per meter

    public static final LEDPattern PULSATING_ORANGE =
        LEDPattern.solid(Color.kOrange)
            .breathe(Constants.Lights.PULSATE_FREQUENCY)
            .blend(LEDPattern.solid(Color.kOrange));

    public static final LEDPattern PULSATING_GREEN =
        LEDPattern.solid(Color.kGreen)
            .breathe(Constants.Lights.PULSATE_FREQUENCY)
            .blend(LEDPattern.solid(Color.kGreen));

    public static final LEDPattern FLASHING_GREEN =
        LEDPattern.solid(Color.kGreen).blink(Constants.Lights.FLASH_FREQUENCY);

    private static HashMap<LEDReader, Long> lastSparkles = new HashMap<>();

    private static LEDPattern sparkle(Frequency frequency, Frequency fadeFrequency) {
      final double periodMicros = frequency.asPeriod().in(Microseconds);
      final double fadePeriodSeconds = fadeFrequency.asPeriod().in(Seconds);
      final double multiplier = MathUtil.clamp(1.0 / fadePeriodSeconds, 0, 1);

      return (reader, writer) -> {
        long now = RobotController.getTime();

        long lastSparkle = lastSparkles.getOrDefault(reader, 0l);

        if (now - lastSparkle > periodMicros) {
          lastSparkle = now;
          lastSparkles.put(reader, lastSparkle);

          int led = (int) (Math.random() * reader.getLength());
          writer.setLED(led, Color.kWhite);
        }

        int baseR, baseG, baseB;

        if (DriverStationUtils.getCurrentAlliance() == Alliance.Red) {
          baseR = 140;
          baseG = 0;
          baseB = 0;
        } else {
          baseR = 0;
          baseG = 0;
          baseB = 140;
        }

        for (int led = 0; led < reader.getLength(); led++) {
          int blendedRGB =
              Color.lerpRGB(
                  reader.getRed(led),
                  reader.getGreen(led),
                  reader.getBlue(led),
                  baseR,
                  baseG,
                  baseB,
                  multiplier);

          writer.setRGB(
              led,
              Color.unpackRGB(blendedRGB, Color.RGBChannel.kRed),
              Color.unpackRGB(blendedRGB, Color.RGBChannel.kGreen),
              Color.unpackRGB(blendedRGB, Color.RGBChannel.kBlue));
        }
      };
    }

    public static final LEDPattern CLIMB_PATTERN = sparkle(Hertz.of(25), Percent.of(5).per(Second));

    public static final Supplier<LEDPattern> ALLIANCE_COLOR =
        () ->
            DriverStationUtils.getCurrentAlliance() == Alliance.Red
                ? LEDPattern.solid(Color.kRed).blend(LEDPattern.solid(Color.kBlack))
                : LEDPattern.solid(Color.kBlue).blend(LEDPattern.solid(Color.kBlack));

    public static final Supplier<LEDPattern> ALLIANCE_COLOR_FANCY =
        () ->
            DriverStationUtils.getCurrentAlliance() == Alliance.Red
                ? LEDPattern.gradient(GradientType.kContinuous, Color.kDarkOrange, Color.kPurple)
                    .mask(LEDPattern.progressMaskLayer(() -> 0.3))
                    .scrollAtAbsoluteSpeed(SCROLL_SPEED.times(5), LED_SPACING)
                    .blend(LEDPattern.solid(Color.kRed))
                : LEDPattern.gradient(GradientType.kContinuous, Color.kViolet, Color.kPurple)
                    .mask(LEDPattern.progressMaskLayer(() -> 0.3))
                    .scrollAtAbsoluteSpeed(SCROLL_SPEED.times(5), LED_SPACING)
                    .blend(LEDPattern.solid(Color.kBlue));

    public static final Supplier<LEDPattern> ALLIANCE_COLOR_FANCY_WITH_CLIMB =
        () -> RobotContainer.isInClimbMode() ? CLIMB_PATTERN : ALLIANCE_COLOR_FANCY.get();

    public static final LEDPattern elevatorPattern =
        LEDPattern.solid(Color.kWhite)
            .mask(
                LEDPattern.progressMaskLayer(
                    () ->
                        RobotContainer.elevator.getCurrentHeight()
                            / Constants.Elevator.MAX_HEIGHT));
    public static final Supplier<LEDPattern> coralIntakePattern =
        () ->
            LEDPattern.solid(
                Color.lerpRGB(
                    Color.kRed,
                    Color.kGreen,
                    (RobotContainer.coralIntake.getArmAngle() - Constants.CoralIntake.ARM_UP)
                        / (Constants.CoralIntake.ARM_DOWN - Constants.CoralIntake.ARM_UP)));
    public static final Supplier<LEDPattern> elevatorHeadPattern =
        () ->
            (RobotContainer.elevatorHead.getCurrentCoralShooterState() == CoralShooterStates.OFF)
                ? PULSATING_GREEN
                : PULSATING_ORANGE;

    public static final Supplier<LEDPattern> sourcePattern =
        () ->
            LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlack)
                .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
    public static final Supplier<LEDPattern> reefScorePattern =
        () ->
            LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kBlack)
                .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
    public static final Supplier<LEDPattern> algaeScorePattern =
        () ->
            LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kPurple, Color.kBlack)
                .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
    public static final Supplier<LEDPattern> cagePattern =
        () ->
            LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kOrange, Color.kBlack)
                .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
    public static final Supplier<LEDPattern> removeAlgaePattern =
        () ->
            LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kBlack)
                .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
    public static final Supplier<LEDPattern> coralScorePattern =
        () ->
            LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kAqua, Color.kBlack)
                .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
    public static final Supplier<LEDPattern> hybridPattern =
        () ->
            LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kBlack)
                .scrollAtAbsoluteSpeed(SCROLL_SPEED, LED_SPACING);
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
    public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.5588;

    public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.5588;

    public static final double FRAME_WIDTH = 0.762;
    public static final double FRAME_WITH_BUMPER_WIDTH = FRAME_WIDTH + Inches.of(6.5).in(Meters);
    public static final double MAX_MECHANISM_HEIGHT = 2.1336;

    public static final double ROBOT_MASS = 64.864; // kg
    public static final double ROBOT_MOI = 14.547; // kg*m^2
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
    public static final Current KRAKEN_DRIVE_SUPPLY_CURRENT_LIMIT = Amps.of(52);

    public static final double FALCON_DRIVE_KS = 0.12373;
    public static final double FALCON_DRIVE_KV = 2.5609;
    public static final double FALCON_DRIVE_KA = 0.10075;
    public static final double FALCON_DRIVE_P = 0.1;

    public static final double FALCON_TURN_KV = 1.7518;
    public static final double FALCON_TURN_KA = 0.015791;
    public static final double FALCON_TURN_KS = 0.08889;
    public static final double FALCON_TURN_P = 1.2;
    public static final double FALCON_TURN_D = 0.2;

    public static final double KRAKEN_DRIVE_KS = 0.089666;
    public static final double KRAKEN_DRIVE_KV = 2.5833;
    public static final double KRAKEN_DRIVE_KA = 0.18188;
    public static final double KRAKEN_DRIVE_P = 0.1;

    public static final double KRAKEN_TURN_KV = 1.2993;
    public static final double KRAKEN_TURN_KA = 0.058972;
    public static final double KRAKEN_TURN_KS = 0.51562;
    public static final double KRAKEN_TURN_P = 55.543;
    public static final double KRAKEN_TURN_D = 2.3952;

    // Wheel diameter
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

    // Turn max velocity and acceleration
    // Calculated from motor rpm 5000 / 60 (rps) / gear ratio (15.43)
    public static final double TurnMaxAngularVelocity = 5; // ROTATIONS / SECOND
    // Calculated from max velocity / time to reach (0.1)
    public static final double TurnMaxAngularAcceleration = 20; // ROTATIONS / SECOND / SECOND

    /** The max speed the robot can travel safely */
    public static final double robotMaxSpeed = 4.35;

    /** The max jerk of the robot below which the pose is certain (in G/s) */
    public static final double MaxPoseCertaintyJerk = 80;

    public static final RobotConfig PPDefaultConfig =
        new RobotConfig(
            Constants.Frame.ROBOT_MASS,
            Constants.Frame.ROBOT_MOI,
            new ModuleConfig(WHEEL_DIAMETER / 2, 4.350, 1.2, DCMotor.getKrakenX60(1), 52, 1),
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation);

    public static final PPHolonomicDriveController PPDriveController =
        new PPHolonomicDriveController(
            new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
            new PIDConstants(1.5, 0.0, 0.0) // Rotation PID constants
            );

    // Module Creation

    /**
     * ===================== NOTE !!!!!! ======================== THESE ARE BACKUP CONSTANTS - NOT
     * USED IF EVERYTHING WORKS EDIT deploy/swerve/motors.json instead
     */

    // #region BACKUP
    public static final ModuleConstants BACKUP_frontLeftConstants =
        new ModuleConstants(2, 3, 0, 0.6353, frontLeftLocation, MotorType.Kraken, MotorType.Kraken);

    public static final ModuleConstants BACKUP_frontRightConstants =
        new ModuleConstants(
            4, 5, 1, 0.8357, frontRightLocation, MotorType.Kraken, MotorType.Kraken);
    public static final ModuleConstants BACKUP_backLeftConstants =
        new ModuleConstants(8, 9, 2, 0.8596, backLeftLocation, MotorType.Kraken, MotorType.Kraken);
    public static final ModuleConstants BACKUP_backRightConstants =
        new ModuleConstants(6, 7, 3, 0.5079, backRightLocation, MotorType.Kraken, MotorType.Kraken);
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

  public final class RobotState {
    public static Mode getMode() {
      return RobotBase.isReal() ? Mode.REAL : (RobotBase.isSimulation() ? Mode.SIM : Mode.REPLAY);
    }

    public static final boolean MOTOR_LOGGING_ENABLED = false;

    public static enum Mode {
      REAL,
      SIM,
      REPLAY
    }
  }
}
