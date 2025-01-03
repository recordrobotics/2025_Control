// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.utils.ModuleConstants;
import frc.robot.utils.ModuleConstants.MotorType;
import java.util.HashMap;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class Shooter {
        public static final double SPEAKER_SPEED = 1;
        public static final double AMP_SPEED = 0.25;
        public static final double REVERSE_SPEED = -0.1;
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

    public final class FieldConstants {

        public static final Translation2d TEAM_RED_SPEAKER = new Translation2d(16, 5.5);
        public static final Translation2d TEAM_BLUE_SPEAKER = new Translation2d(0.6, 5.6);
        public static final Rotation2d TEAM_RED_AMP = Rotation2d.fromDegrees(90); // new Translation2d(14.7, 8.5);
        public static final Rotation2d TEAM_BLUE_AMP = Rotation2d.fromDegrees(90); // new Translation2d(2.775, 8.5);

        // Field width and length
        public static final double FIELD_X_DIMENSION = 16.54; // Length
        public static final double FIELD_Y_DIMENSION = 8.21; // Width
    }

    public final class Control {

        // Sensitivity for speed meter
        public static final double DIRECTIONAL_SPEED_METER_LOW = 0.25;
        public static final double DIRECTIONAL_SPEED_METER_HIGH = 4.0;
        public static final double SPIN_SPEED_METER_LOW = 0.5;
        public static final double SPIN_SPEED_METER_HIGH = 4.3;

        // Sensitivies for directional controls (XY) and spin (theta)
        public static final double JOSYSTICK_DIRECTIONAL_SENSITIVITY = 1;
        public static final double JOYSTICK_SPIN_SENSITIVITY = 1;
        public static final double JOYSTICK_X_THRESHOLD = 0.15;
        public static final double JOYSTICK_Y_THRESHOLD = 0.15;
        public static final double JOYSTICK_SPIN_THRESHOLD = 0.76;

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
            public static final double STEEPNESS = 2.6; // Linear = 1, <1 = faster scaling, >1 = slower scaling
        }
    }

    public final class Swerve {

        // Works out module locations
        private static final double locX = Constants.Frame.ROBOT_WHEEL_DISTANCE_WIDTH / 2;
        private static final double locY = Constants.Frame.ROBOT_WHEEL_DISTANCE_LENGTH / 2;
        public static final double locDist = Math.sqrt(locX * locX + locY * locY);

        private static final Translation2d frontLeftLocation = new Translation2d(locX, locY);
        private static final Translation2d frontRightLocation = new Translation2d(locX, -locY);
        private static final Translation2d backLeftLocation = new Translation2d(-locX, locY);
        private static final Translation2d backRightLocation = new Translation2d(-locX, -locY);

        // Gear ratios for falcon and kraken
        public static final double FALCON_TURN_GEAR_RATIO = 15.43; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)
        public static final double FALCON_DRIVE_GEAR_RATIO = 7.36; // (https://web.archive.org/web/20230117081053/https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options)

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

        public static final double KRAKEN_DRIVE_FEEDFORWARD_KS = 0.1205;
        public static final double KRAKEN_DRIVE_FEEDFORWARD_KV = 2.4915;

        // Same between Falcon and Kraken since they share the same encoders
        public static final double RELATIVE_ENCODER_RATIO = 2048;

        // Wheel diameter
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

        // Turn & Drive max velocity and acceleration
        public static final double TurnMaxAngularVelocity = 25; // Drivetrain.kMaxAngularSpeed;
        public static final double TurnMaxAngularAcceleration = 34; // 2 * Math.PI; // radians per second squared
        public static final double DriveMaxAngularVelocity = 15; // Drivetrain.kMaxAngularSpeed;
        public static final double DriveMaxAngularAcceleration = 30; // 2 * Math.PI; // radians per second squared

        /** The max speed the robot is allowed to travel */
        public static final double robotMaxSpeed = 7.0;

        public static final HolonomicPathFollowerConfig PathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(1, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1, 0.0, 0.0), // Rotation PID constants
                3, // Max module speed, in m/s
                locDist, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        );

        // Module Creation
        public static final ModuleConstants frontLeftConstants = new ModuleConstants(
                2,
                1,
                1,
                0.620, // 0.12363, //0.597
                frontLeftLocation,
                MotorType.Falcon,
                MotorType.Falcon);
        public static final ModuleConstants frontRightConstants = new ModuleConstants(
                4,
                3,
                2,
                0.924, // 0.41599, //0.886
                frontRightLocation,
                MotorType.Falcon,
                MotorType.Falcon);
        public static final ModuleConstants backLeftConstants = new ModuleConstants(
                8,
                7,
                4,
                0.676, // 0.17245, //0.857
                backLeftLocation,
                MotorType.Falcon,
                MotorType.Falcon);
        public static final ModuleConstants backRightConstants = new ModuleConstants(
                6,
                5,
                3,
                0.371, // 0.86499, //0.556
                backRightLocation,
                MotorType.Falcon,
                MotorType.Falcon);
    }

    public final class Frame {

        /**
         * Distance between wheels (width aka between left and right and length aka
         * between front and back).
         * Used for calculating wheel locations on the robot
         */
        public static final double ROBOT_WHEEL_DISTANCE_WIDTH = 0.46;
        public static final double ROBOT_WHEEL_DISTANCE_LENGTH = 0.46;
    }


    public final class Vision {

        public static final String cameraID = new String("photonvision");

        //The offset from the center of the robot to the camera, and from facing exactly forward to the orientation of the camera.
	    public static final Transform3d robotToCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(11), -1*Units.inchesToMeters(9), 0.1725), 
            new Rotation3d(0,0,0)
        ); 

        public static final Transform3d[] tagTransforms = {//april tags 1-8 in order. values contained are x, y, z, theta, in that order. x, y, z are distances in meters, theta is in radians.
            new Transform3d(new Translation3d(15.513558, 1.071626, 0.462788), new Rotation3d(0,0,Math.PI)),
            new Transform3d(new Translation3d(15.513558, 2.748026, 0.462788), new Rotation3d(0,0,Math.PI)),
            new Transform3d(new Translation3d(15.513558, 4.424426, 0.462788), new Rotation3d(0,0,Math.PI)),
            new Transform3d(new Translation3d(16.178784, 6.749796, 0.695452), new Rotation3d(0,0,Math.PI)),
            new Transform3d(new Translation3d(0.36195, 6.749796, 0.695452), new Rotation3d(0,0,0)),
            // This transform below is the only one that is currently relevant.
            new Transform3d(new Translation3d(1.8415, 8.2042, 1.355852), new Rotation3d(0,0,4.71239)),
            new Transform3d(new Translation3d(1.02743, 2.748026, 0.462788), new Rotation3d(0,0,0)),
            new Transform3d(new Translation3d(1.02743, 1.071626, 0.462788), new Rotation3d(0,0,0))
        };

        public static final String leftCameraID = null;

        public static final String rightCameraID = null;

        public static final double cameraWidth = 0;
    }
}