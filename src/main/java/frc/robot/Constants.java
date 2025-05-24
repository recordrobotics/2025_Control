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
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.MotorLocation;
import frc.robot.utils.ModuleConstants.MotorType;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdRoutine;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
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
  public final class Game {
    public static final AprilTagFieldLayout APRILTAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public interface IGamePosition {
      public Pose2d getPose();

      public static <E extends IGamePosition> E closestTo(Pose2d pose, E[] values) {
        E closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (E pos : values) {
          double distance = pos.getPose().getTranslation().getDistance(pose.getTranslation());
          if (distance < closestDistance) {
            closest = pos;
            closestDistance = distance;
          }
        }
        return closest;
      }

      public static Pose2d[] aggregatePositions(IGamePosition[]... values) {
        List<Pose2d> poses = new ArrayList<>();
        for (IGamePosition[] value : values) {
          for (IGamePosition pos : value) {
            poses.add(pos.getPose());
          }
        }
        return poses.toArray(new Pose2d[0]);
      }
    }

    public static enum AlgaeLevel {
      LOW,
      HIGH
    }

    public static final Distance ALGAE_OFFSET = Meters.of(-0.1055532306);

    public static enum AlgaePosition implements IGamePosition {
      BlueAB(AlgaeLevel.HIGH, 18),
      BlueCD(AlgaeLevel.LOW, 17),
      BlueEF(AlgaeLevel.HIGH, 22),
      BlueGH(AlgaeLevel.LOW, 21),
      BlueIJ(AlgaeLevel.HIGH, 20),
      BlueKL(AlgaeLevel.LOW, 19),

      RedAB(AlgaeLevel.HIGH, 7),
      RedCD(AlgaeLevel.LOW, 8),
      RedEF(AlgaeLevel.HIGH, 9),
      RedGH(AlgaeLevel.LOW, 10),
      RedIJ(AlgaeLevel.HIGH, 11),
      RedKL(AlgaeLevel.LOW, 6);

      private AlgaeLevel level;
      private Pose2d pose;

      private AlgaePosition(AlgaeLevel level, int apriltagId) {
        this.level = level;
        this.pose = calculatePoseFromAprilTag(apriltagId);
      }

      private static Pose2d calculatePoseFromAprilTag(int apriltagId) {
        return APRILTAG_LAYOUT
            .getTagPose(apriltagId)
            .get()
            .toPose2d()
            .transformBy(
                new Transform2d(
                    Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2 + Centimeters.of(6).in(Meters),
                    ALGAE_OFFSET.in(Meters),
                    Rotation2d.k180deg));
      }

      public AlgaeLevel getLevel() {
        return level;
      }

      public Pose2d getPose() {
        return pose;
      }
    }

    public static enum CoralLevel {
      L1(ElevatorHeight.L1),
      L2(ElevatorHeight.L2),
      L3(ElevatorHeight.L3),
      L4(ElevatorHeight.L4);

      private ElevatorHeight height;

      private CoralLevel(ElevatorHeight height) {
        this.height = height;
      }

      public ElevatorHeight getHeight() {
        return height;
      }
    }

    private static Distance REEF_SEGMENT_OFFSET = Meters.of(0.1743095);
    private static Distance SHOOTER_OFFSET = Meters.of(0.182088);

    public static enum CoralPosition implements IGamePosition {
      BlueA(18, 0),
      BlueB(18, 1),
      BlueC(17, 0),
      BlueD(17, 1),
      BlueE(22, 0),
      BlueF(22, 1),
      BlueG(21, 0),
      BlueH(21, 1),
      BlueI(20, 0),
      BlueJ(20, 1),
      BlueK(19, 0),
      BlueL(19, 1),

      RedA(7, 0),
      RedB(7, 1),
      RedC(8, 0),
      RedD(8, 1),
      RedE(9, 0),
      RedF(9, 1),
      RedG(10, 0),
      RedH(10, 1),
      RedI(11, 0),
      RedJ(11, 1),
      RedK(6, 0),
      RedL(6, 1);

      private Pose2d pose;
      public final int apriltagId;

      private CoralPosition(int apriltagId, int side) {
        this.pose = calculatePoseFromAprilTag(apriltagId, side);
        this.apriltagId = apriltagId;
      }

      private static Pose2d calculatePoseFromAprilTag(int apriltagId, int side) {
        return APRILTAG_LAYOUT
            .getTagPose(apriltagId)
            .get()
            .toPose2d()
            .transformBy(
                new Transform2d(
                    Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2,
                    (side == 0 ? -REEF_SEGMENT_OFFSET.in(Meters) : REEF_SEGMENT_OFFSET.in(Meters))
                        + SHOOTER_OFFSET.in(Meters),
                    Rotation2d.k180deg));
      }

      public Pose2d getFirstStagePose() {
        return pose.transformBy(new Transform2d(-0.2, 0, Rotation2d.kZero));
      }

      public Pose2d getPose() {
        return getFirstStagePose();
      }

      public Pose2d getPose(CoralLevel level) {
        switch (level) {
          case L1:
            return pose.transformBy(new Transform2d(-0.1, 0, Rotation2d.kCW_90deg));
          case L2:
          case L3:
            return pose.transformBy(new Transform2d(-0.05, 0, Rotation2d.kZero));
          case L4:
            return pose.transformBy(new Transform2d(-0.1, 0, Rotation2d.kZero));
          default:
            return pose; // this is bad
        }
      }
    }

    public static enum SourceCoralSpawnPosition implements IGamePosition {
      // Coral is only used for simulation, no point in making a april tag version
      BlueCoralLeft(new Pose2d(0.702, 7.545, Rotation2d.fromDegrees(215.988608))),
      BlueCoralRight(new Pose2d(0.648, 0.489, Rotation2d.fromDegrees(-35.988608))),
      RedCoralLeft(BlueCoralLeft),
      RedCoralRight(BlueCoralRight);

      private Pose2d pose;

      private SourceCoralSpawnPosition(Pose2d pose) {
        this.pose = pose;
      }

      private SourceCoralSpawnPosition(SourceCoralSpawnPosition blueSide) {
        this.pose = FlippingUtil.flipFieldPose(blueSide.getPose());
      }

      public Pose2d getPose() {
        return pose;
      }
    }

    public static enum SourcePosition implements IGamePosition {
      BlueOuterLeft(13, 1),
      BlueOuterRight(12, 1),
      BlueInnerLeft(13, 0),
      BlueInnerRight(12, 0),

      RedOuterLeft(1, 1),
      RedOuterRight(2, 1),
      RedInnerLeft(1, 0),
      RedInnerRight(2, 0);

      private Pose2d pose;

      private SourcePosition(Pose2d pose) {
        this.pose = pose;
      }

      private SourcePosition(int apriltagId, int side) {
        this.pose = calculatePoseFromAprilTag(apriltagId, side);
      }

      private static Pose2d calculatePoseFromAprilTag(int apriltagId, int side) {
        return APRILTAG_LAYOUT
            .getTagPose(apriltagId)
            .get()
            .toPose2d()
            .transformBy(
                new Transform2d(
                    Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2,
                    (side == 0
                        ? -Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2
                        : Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2),
                    Rotation2d.kCCW_90deg));
      }

      public Pose2d getPose() {
        return pose;
      }
    }

    public static enum ProcessorPosition implements IGamePosition {
      BlueProcessor(16),
      RedProcessor(3);

      private Pose2d pose;

      private ProcessorPosition(int apriltagId) {
        this.pose = calculatePoseFromAprilTag(apriltagId);
      }

      private static Pose2d calculatePoseFromAprilTag(int apriltagId) {
        return APRILTAG_LAYOUT
            .getTagPose(apriltagId)
            .get()
            .toPose2d()
            .transformBy(
                new Transform2d(
                    Constants.Frame.FRAME_WITH_BUMPER_WIDTH / 2, 0, Rotation2d.k180deg));
      }

      public Pose2d getPose() {
        return pose;
      }
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

  public final class Align {
    public static final double MAX_VELOCITY = 1.5; // m/s
    public static final double MAX_ACCELERATION = 10.0; // m/s^2
    public static final double MAX_ANGULAR_VELOCITY = 4.0; // rad/s
    public static final double MAX_ANGULAR_ACCELERATION = 7.0; // rad/s^2

    public static final double translationalTolerance = 0.01; // Meters
    public static final double rotationalTolerance = 0.02; // Radians

    public static final double MAX_REEF_ALIGN_DISTANCE = 2.5; // Meters

    public static final PathConstraints PATH_CONSTRAINTS =
        new PathConstraints(
            MAX_VELOCITY, MAX_ACCELERATION, MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);
  }

  public final class PhotonVision {
    public static final String PHOTON_L1_NAME = "photon-l1";
    public static final String PHOTON_SOURCE_NAME = "photon-source";
    public static final String PHOTON_CORAL_INTAKE = "coral-intake";

    public static final Transform3d l1TransformRobotToCamera =
        new Transform3d(
            new Translation3d(-0.031750, -0.373120, 0.196097),
            new Rotation3d(0, Units.degreesToRadians(-12.894), Units.degreesToRadians(90)));
    public static final Transform3d sourceTransformRobotToCamera =
        new Transform3d(
            new Translation3d(-0.006350, 0.370769, 0.197020),
            new Rotation3d(0, Units.degreesToRadians(-18.951), Units.degreesToRadians(-90)));
    public static final Transform3d groundIntakeTransformRobotToCamera =
        new Transform3d(
            new Translation3d(Meters.of(0.142850), Meters.of(0.369435), Meters.of(0.526114)),
            new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(90 + 25)));

    public static final Distance CORAL_ID_DISTANCE = Inches.of(8);
    public static final Time CORAL_TIMEOUT = Seconds.of(0.5);

    public static final int CORAL_ID = 1;

    public static final double ROT_STD_DEV_WHEN_TRUSTING = 4;
  }

  public final class HybridConstants {
    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will
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

    public static final double MAX_ARM_VELOCITY = 10;
    public static final double MAX_ARM_ACCELERATION = 15;

    public static final double kP = 14.403;
    public static final double kI = 0;
    public static final double kD = 0.1524;
    public static final double kG = 0;
    public static final double kS = 0.02808;
    public static final double kV = 5.4441;
    public static final double kA = 0.1539;

    public static final double CLIMB_VOLTAGE_SLEW_RATE =
        12.0 / 0.3; /* Go from 0 - 12 in 0.3 seconds */

    public static final double CLIMB_EXPECTED_KV_MIN = 6.0;
    public static final Time CLIMB_EXPECTED_KV_TIMEOUT = Seconds.of(0.9);

    public static final Angle START_ROTATIONS = Rotations.of(0);
    public static final Angle PARK_ROTATIONS = START_ROTATIONS;
    public static final Angle EXTENDED_ROTATIONS = Rotations.of(-1.2);
    public static final Angle CLIMBED_ROTATIONS = Rotations.of(0.1);

    public static final double RATCHET_ENGAGED = 0.15;
    public static final double RATCHET_DISENGAGED = 0.025;

    public static final double GEAR_RATIO = 48; // 48:1 gearbox

    // TODO: make correct
    public static final Pose2d ROOT_MECHANISM_POSE = new Pose2d(0, 0.4, Rotation2d.fromDegrees(0));
    // TODO: make correct
    public static final Distance LENGTH = Meters.of(0.431);
    public static final double ANGLE_OFFSET = 0;
  }

  public final class ElevatorArm {
    public static final Current ARM_SUPPLY_CURRENT_LIMIT = Amps.of(15);
    public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(60);

    public static final double MAX_ARM_VELOCITY = 12;
    public static final double MAX_ARM_ACCELERATION = 40;

    public static final double kP = 43.839;
    public static final double kI = 0;
    public static final double kD = 4.1082;
    public static final double kG = 0.53175;
    public static final double kS = 0.17279;
    public static final double kV = 3.1543;
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

  public final class Elevator {
    public static final double kDt = 0.02;
    public static final double kMaxVelocity = 2.3;
    public static final double kMaxAcceleration = 10;
    public static final double kV = 6.8;
    public static final double kA = 0.21006;
    public static final double kG = 0.20027;
    public static final double kS = 0.016248;

    public static final double kP = 68.424;
    public static final double kI = 0.0;
    public static final double kD = 9.2646;

    public static final double STARTING_HEIGHT = 0;
    public static final double LOWEST_HOLD_HEIGHT = Units.inchesToMeters(1.25);

    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);
    public static final Current SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(30);
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(100);

    public static final double GEAR_RATIO = 13.4321 / 2.0 /* compensation for second stage ratio */;
    public static final double DRUM_RADIUS = Units.inchesToMeters(0.8783343);
    // 2 * pi * r / gear ratio because same as getting distance a wheel moved, just
    // vertically
    public static final double METERS_PER_ROTATION = DRUM_RADIUS * 2 * Math.PI / GEAR_RATIO;

    public static final double AT_GOAL_POSITION_TOLERANCE = 0.03;
    public static final double AT_GOAL_VELOCITY_TOLERANCE = 0.63514;

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

    public static final double arm_kP = 68.721;
    public static final double arm_kI = 0;
    public static final double arm_kD = 8.7176;
    public static final double arm_kG = 0.47407;
    public static final double arm_kS = 0.41583;
    public static final double arm_kV = 5.3;
    public static final double arm_kA = 0.1;

    public static final double ARM_UP = Units.degreesToRadians(80.19);
    public static final double ARM_PUSH = Units.degreesToRadians(84.19);
    public static final double ARM_INTAKE = Units.degreesToRadians(77.08);
    public static final double ARM_SCORE_L1 = Units.degreesToRadians(22.35);
    public static final double ARM_DOWN = Units.degreesToRadians(-49.5);
    public static final double ARM_START_POS =
        SysIdManager.getSysIdRoutine() == SysIdRoutine.CoralIntakeArm
            ? ARM_DOWN
            : Units.degreesToRadians(89.3);

    public static final double ARM_GEAR_RATIO = 56.8889; // 16:1 * 64/18

    public static final double WHEEL_GEAR_RATIO = 10;

    public static final double SHOOT_TIME = 0.3;

    public static final Distance INTAKE_X_OFFSET = Inches.of(-3.3999);

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
        () ->
            RobotContainer.climber.getCurrentState() != ClimberState.Park
                ? CLIMB_PATTERN
                : ALLIANCE_COLOR_FANCY.get();

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

    public static final double FRAME_WIDTH = Inches.of(30).in(Meters);
    public static final double FRAME_WITH_BUMPER_WIDTH = FRAME_WIDTH + Inches.of(6.5).in(Meters);
    public static final double MAX_MECHANISM_HEIGHT = 2.1336;

    public static final double ROBOT_MASS = 64.864; // kg
    public static final double ROBOT_MOI = 14.547; // kg*m^2
  }

  public final class Swerve {

    public static double kDt = 0.020;

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

  public final class Auto {
    public static final Time SOURCE_TIMEOUT = Seconds.of(0.8);
  }

  public final class RobotState {
    public static Mode getMode() {
      return RobotBase.isReal() ? Mode.REAL : (RobotBase.isSimulation() ? Mode.SIM : Mode.REPLAY);
    }

    public static final boolean MOTOR_LOGGING_ENABLED = false;

    public static final AutoLogLevel.Level AUTO_LOG_LEVEL =
        RobotBase.isReal()
            ? (SysIdManager.getSysIdRoutine() != SysIdRoutine.None
                ? AutoLogLevel.Level.Sysid
                : AutoLogLevel.Level.Real)
            : AutoLogLevel.Level.Sim;

    public static final VisionSimulationMode VISION_SIMULATION_MODE =
        VisionSimulationMode.MAPLE_CLEAN;

    public static enum VisionSimulationMode {
      PHOTON_SIM,
      MAPLE_CLEAN,
      MAPLE_NOISE;
    }

    public static enum Mode {
      REAL,
      SIM,
      REPLAY
    }
  }
}
