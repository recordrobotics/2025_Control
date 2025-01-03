// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Swerve extends SubsystemBase {

        /**
         * The number of wheels on the robot
         */
        private final int wheelCount = Constants.Swerve.SWERVE_WHEEL_COUNT;

        /**
         * Motors that spin the wheels to move the robot
         */
        private TalonFX[] speedMotors = new TalonFX[wheelCount];

        /**
         * Motors that rotate the wheels to change the direction of movement of the
         * robot
         */
        private TalonFX[] directionMotors = new TalonFX[wheelCount];

        /**
         * Encoders for initializing the direction motor's position
         */
        private DutyCycleEncoder[] encoders = new DutyCycleEncoder[wheelCount];

        private DutyCycleEncoderSim[] encoderSims = new DutyCycleEncoderSim[wheelCount];

        /**
         * Direction motor PIDs
         */
        private PIDController[] directionPID = new PIDController[wheelCount];

        /**
         * Target swerve module states that are updated and optimized in periodic
         */
        private SwerveModuleState[] targetStates = new SwerveModuleState[wheelCount];

        private DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
                        DCMotor.getFalcon500(2), // 2 on each side
                        Constants.Swerve.SPEED_GEAR_RATIO,
                        7.5, // MOI of 7.5 kg m^2 (from CAD model).
                        Units.lbsToKilograms(60.0), // The mass of the robot is 60 lbs.
                        Constants.Swerve.SWERVE_WHEEL_DIAMETER / 2,
                        Constants.Swerve.ROBOT_WHEEL_DISTANCE_WIDTH,
                        null);

        /**
         * Locations of the wheels on the robot frame.
         */
        Translation2d[] wheelLocations = {
                        new Translation2d(Constants.Swerve.ROBOT_WHEEL_DISTANCE_WIDTH / 2,
                                        Constants.Swerve.ROBOT_WHEEL_DISTANCE_LENGTH / 2),
                        new Translation2d(Constants.Swerve.ROBOT_WHEEL_DISTANCE_WIDTH / 2,
                                        -(Constants.Swerve.ROBOT_WHEEL_DISTANCE_LENGTH / 2)),
                        new Translation2d(-(Constants.Swerve.ROBOT_WHEEL_DISTANCE_WIDTH / 2),
                                        Constants.Swerve.ROBOT_WHEEL_DISTANCE_LENGTH / 2),
                        new Translation2d(-(Constants.Swerve.ROBOT_WHEEL_DISTANCE_WIDTH / 2),
                                        -(Constants.Swerve.ROBOT_WHEEL_DISTANCE_LENGTH / 2)),
        };

        /**
         * Kinematics for the swerve drive with the four wheel locations. Used in
         * periodic to calculate the target wheel states.
         */
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        wheelLocations[0],
                        wheelLocations[1],
                        wheelLocations[2],
                        wheelLocations[3]);

        /**
         * Target Velocity and Angle of the chassis
         */
        ChassisSpeeds targetChassisSpeed = new ChassisSpeeds();

        // Nav
        private NavSensor _nav = new NavSensor();

        // Creates swerve post estimation filter
        public SwerveDrivePoseEstimator poseFilter;
        // accurate values

        public Swerve() {
                // Create motor objects
                for (int i = 0; i < wheelCount; i++) {
                        // Create new TalonFX objects for each speed and direction motor
                        speedMotors[i] = new TalonFX(RobotMap.swerve.SPEED_MOTOR_DEVICE_IDS[i]);
                        directionMotors[i] = new TalonFX(RobotMap.swerve.DIRECTION_MOTOR_DEVICE_IDS[i]);
                        // Create new DutyCycleEncoder objects for each encoder
                        encoders[i] = new DutyCycleEncoder(RobotMap.swerve.ENCODER_DEVICE_IDS[i]);
                        encoderSims[i] = new DutyCycleEncoderSim(encoders[i]);
                        // Create new PIDController objects for each direction motor
                        directionPID[i] = new PIDController(Constants.Swerve.DIRECTION_KP,
                                        Constants.Swerve.DIRECTION_KI,
                                        Constants.Swerve.DIRECTION_KD);
                        // Create new SwerveModuleState objects for each wheel
                        targetStates[i] = new SwerveModuleState();
                }

                // Worried about latency in reading of some values so wait a few seconds
                Timer.delay(5);

                // Init the motor and PID values
                for (int i = 0; i < wheelCount; i++) {
                        // Reset motor speed
                        speedMotors[i].set(0);
                        directionMotors[i].set(0);

                        // Offset direction motor encoder position
                        final double encoderValue = getEncoderPosition(i);


                        //final double encoderValueWithRatio = -encoderValue * Constants.Swerve.DIRECTION_GEAR_RATIO;
                        // Set direction motor position offset
                        //directionMotors[i].setPosition(encoderValueWithRatio);
                        //directionPID[i].enableContinuousInput(-0.5, 0.5);
                }

                
        }

        /**
         * Gets the absolute encoder position offsetted using ENCODER_OFFSETS
         * 
         * @param encoderIndex index of encoder in array
         * @return the absolute position of the encoder relative to our our robots zero
         */
        private double getEncoderPosition(int encoderIndex) {
                return encoders[encoderIndex].getAbsolutePosition();
        }


        @Override
        public void periodic() {

                for (int i = 0; i < wheelCount; i++) {
                        // Reset motor speed
                        speedMotors[i].set(0);
                        directionMotors[i].set(0);

                        // Offset direction motor encoder position
                        final double encoderValue = getEncoderPosition(i);

                        SmartDashboard.putNumber("AbEnc:" + RobotMap.swerve.ENCODER_DEVICE_IDS[i], encoderValue);

                        //final double encoderValueWithRatio = -encoderValue * Constants.Swerve.DIRECTION_GEAR_RATIO;
                        // Set direction motor position offset
                        //directionMotors[i].setPosition(encoderValueWithRatio);
                        //directionPID[i].enableContinuousInput(-0.5, 0.5);
                }

                }

        

        @Override
        public void simulationPeriodic() {
                periodic();

                // Broken drivetrain simulation
                drivetrainSim.setInputs(speedMotors[0].get() * RobotController.getInputVoltage(),
                                speedMotors[2].get() * RobotController.getInputVoltage());

                drivetrainSim.update(0.02);

                for (int i = 0; i < wheelCount; i++) {
                        if (i < 2) {
                                encoderSims[i].setDistance(drivetrainSim.getLeftPositionMeters());
                                speedMotors[i].getSimState()
                                                .setRawRotorPosition(drivetrainSim.getLeftPositionMeters() * 100);
                                speedMotors[i].getSimState()
                                                .setRotorVelocity(drivetrainSim.getLeftVelocityMetersPerSecond() * 100);
                        } else {
                                encoderSims[i].setDistance(drivetrainSim.getRightPositionMeters());
                                speedMotors[i].getSimState()
                                                .setRawRotorPosition(drivetrainSim.getRightPositionMeters() * 100);
                                speedMotors[i].getSimState()
                                                .setRotorVelocity(
                                                                drivetrainSim.getRightVelocityMetersPerSecond() * 100);
                        }
                }
        }
}
