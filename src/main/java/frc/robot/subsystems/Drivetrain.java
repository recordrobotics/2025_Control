// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DriveCommandData;
import frc.robot.Constants;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ShuffleboardField;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

        // Creates Nav object
        private final NavSensor _nav = new NavSensor();

        // Creates swerve module objects
        private final SwerveModule m_frontLeft = new SwerveModule(Constants.Swerve.frontLeftConstants);
        private final SwerveModule m_frontRight = new SwerveModule(Constants.Swerve.frontRightConstants);
        private final SwerveModule m_backLeft = new SwerveModule(Constants.Swerve.backLeftConstants);
        private final SwerveModule m_backRight = new SwerveModule(Constants.Swerve.backRightConstants);

        // Creates swerve kinematics
        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        Constants.Swerve.frontLeftConstants.wheelLocation,
                        Constants.Swerve.frontRightConstants.wheelLocation,
                        Constants.Swerve.backLeftConstants.wheelLocation,
                        Constants.Swerve.backRightConstants.wheelLocation);

        // Creates swerve post estimation filter
        public SwerveDrivePoseEstimator poseFilter;

        // Init drivetrain
        public Drivetrain() {
                _nav.resetAngleAdjustment();

                poseFilter = new SwerveDrivePoseEstimator(
                                m_kinematics,
                                _nav.getAdjustedAngle(),
                                new SwerveModulePosition[] {
                                                m_frontLeft.getModulePosition(),
                                                m_frontRight.getModulePosition(),
                                                m_backLeft.getModulePosition(),
                                                m_backRight.getModulePosition()
                                },
                                DriverStationUtils.getStartingLocation().getPose());
        }

        /**
         * Method to drive the robot using joystick info.
         *
         * @param xSpeed        Speed of the robot in the x direction (forward).
         * @param ySpeed        Speed of the robot in the y direction (sideways).
         * @param rot           Angular rate of the robot.
         * @param fieldRelative Whether the provided x and y speeds are relative to the
         *                      field.
         */
        public void drive(DriveCommandData driveCommandData) {
                // Data from driveCommandData
                boolean fieldRelative = driveCommandData.fieldRelative;
                double xSpeed = driveCommandData.xSpeed;
                double ySpeed = driveCommandData.ySpeed;
                double rot = driveCommandData.rot;

                // Calculates swerveModuleStates given optimal ChassisSpeeds given by control
                // scheme

                SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                                fieldRelative
                                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                                poseFilter.getEstimatedPosition().getRotation())
                                                : new ChassisSpeeds(xSpeed, ySpeed, rot));

                // Desaturates wheel speeds
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.robotMaxSpeed);

                // Sets state for each module
                m_frontLeft.setDesiredState(swerveModuleStates[0]);
                m_frontRight.setDesiredState(swerveModuleStates[1]);
                m_backLeft.setDesiredState(swerveModuleStates[2]);
                m_backRight.setDesiredState(swerveModuleStates[3]);
        }

        // set PID target to 0 but also immediately stop all modules
        public void stop() {
                drive(new DriveCommandData(0, 0, 0, false));
                m_frontLeft.stop();
                m_frontRight.stop();
                m_backLeft.stop();
                m_backRight.stop();
        }

        @Override
        public void periodic() {
                poseFilter.update(
                                _nav.getAdjustedAngle(),
                                new SwerveModulePosition[] {
                                                m_frontLeft.getModulePosition(),
                                                m_frontRight.getModulePosition(),
                                                m_backLeft.getModulePosition(),
                                                m_backRight.getModulePosition()
                                });
                ShuffleboardField.setRobotPose(poseFilter.getEstimatedPosition());
        }

        /** Resets the field relative position of the robot (mostly for testing). */
        public void resetPose() {
                _nav.resetAngleAdjustment();
                m_frontLeft.resetDriveMotorPosition();
                m_frontRight.resetDriveMotorPosition();
                m_backLeft.resetDriveMotorPosition();
                m_backRight.resetDriveMotorPosition();
                poseFilter.resetPosition(
                                _nav.getAdjustedAngle(),
                                new SwerveModulePosition[] {
                                                m_frontLeft.getModulePosition(),
                                                m_frontRight.getModulePosition(),
                                                m_backLeft.getModulePosition(),
                                                m_backRight.getModulePosition()
                                },
                                DriverStationUtils.getStartingLocation().getPose());
        }

        /**
         * Returns the current robot relative chassis speeds of the swerve kinematics
         */
        public ChassisSpeeds getRobotRelativeSpeeds() {
                return m_kinematics.toChassisSpeeds(
                                m_frontLeft.getModuleState(),
                                m_frontRight.getModuleState(),
                                m_backLeft.getModuleState(),
                                m_backRight.getModuleState());
        }

        /**
         * Similar to resetPose but adds an argument for the initial pose
         */
        public void setToPose(Pose2d pose) {
                poseFilter.resetPosition(_nav.getAdjustedAngle(),
                                new SwerveModulePosition[] {
                                                m_frontLeft.getModulePosition(),
                                                m_frontRight.getModulePosition(),
                                                m_backLeft.getModulePosition(),
                                                m_backRight.getModulePosition()
                                }, pose);
        }
}