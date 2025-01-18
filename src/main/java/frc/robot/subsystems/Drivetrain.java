package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.manual.ManualSwerve;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.DriveCommandDataAutoLogged;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.Logger;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends KillableSubsystem implements ShuffleboardPublisher {
  // Creates swerve module objects
  private final SwerveModule m_frontLeft = new SwerveModule(Constants.Swerve.frontLeftConstants);
  private final SwerveModule m_frontRight = new SwerveModule(Constants.Swerve.frontRightConstants);
  private final SwerveModule m_backLeft = new SwerveModule(Constants.Swerve.backLeftConstants);
  private final SwerveModule m_backRight = new SwerveModule(Constants.Swerve.backRightConstants);

  private DriveCommandDataAutoLogged driveCommandDataAutoLogged = new DriveCommandDataAutoLogged();

  // Creates swerve kinematics
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          Constants.Swerve.frontLeftConstants.wheelLocation,
          Constants.Swerve.frontRightConstants.wheelLocation,
          Constants.Swerve.backLeftConstants.wheelLocation,
          Constants.Swerve.backRightConstants.wheelLocation);

  public Drivetrain() {
    setDefaultCommand(new ManualSwerve());
  
    sysIdRoutineDriveMotors =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state -> Logger.recordOutput("SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(
                this::SysIdOnlyDriveMotors,
                // Tell SysId how to record a frame of data
                log -> {
                  log.motor("drivetrain-drive-motor")
                      .voltage(
                          m_appliedVoltageDriveMotors.mut_replace(
                              SysIdOnlyGetDriveMotorVolts() * RobotController.getBatteryVoltage(),
                              Volts))
                      .linearPosition(
                          m_angleDriveMotors.mut_replace(SysIdOnlyGetDriveMotorPosition(), Meters))
                      .linearVelocity(
                          m_velocityDriveMotors.mut_replace(
                              SysIdOnlyGetDriveMotorVelocity(), MetersPerSecond));
                },
                this));

    sysIdRoutineTurnMotors =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state -> Logger.recordOutput("SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(
                this::SysIdOnlyTurnMotors,
                // Tell SysId how to record a frame of data
                log -> {
                  log.motor("drivetrain-turn-motor")
                      .voltage(
                          m_appliedVoltageTurnMotors.mut_replace(
                              SysIdOnlyGetTurnMotorVolts() * RobotController.getBatteryVoltage(),
                              Volts))
                      .angularPosition(
                          m_angleTurnMotors.mut_replace(SysIdOnlyGetTurnMotorPosition(), Rotations))
                      .angularVelocity(
                          m_velocityTurnMotors.mut_replace(
                              SysIdOnlyGetTurnMotorVelocity(), RotationsPerSecond));
                },
                this));
  }

  private final MutVoltage m_appliedVoltageDriveMotors = Volts.mutable(0);
  private final MutDistance m_angleDriveMotors = Meters.mutable(0);
  private final MutLinearVelocity m_velocityDriveMotors = MetersPerSecond.mutable(0);

  private final SysIdRoutine sysIdRoutineDriveMotors;

  private final MutVoltage m_appliedVoltageTurnMotors = Volts.mutable(0);
  private final MutAngle m_angleTurnMotors = Radians.mutable(0);
  private final MutAngularVelocity m_velocityTurnMotors = RadiansPerSecond.mutable(0);

  private final SysIdRoutine sysIdRoutineTurnMotors;

  /**
   * Drives the robot using joystick info.
   *
   * @param driveCommandData contains all the info to drive the robot. The xSpeed and ySpeed
   *     components are relative to the robot's current orientation if fieldRelative is false, and
   *     relative to the field if fieldRelative is true.
   * @param currentRotation the current rotation of the robot in the field coordinate system. Used
   *     to convert the joystick x and y components to the field coordinate system if fieldRelative
   *     is true.
   */
  public void drive(DriveCommandData driveCommandData) {
    // Data from driveCommandData
    boolean fieldRelative = driveCommandData.fieldRelative;
    double xSpeed = driveCommandData.xSpeed;
    double ySpeed = driveCommandData.ySpeed;
    double rot = driveCommandData.rot;

    // Calculates swerveModuleStates given optimal ChassisSpeeds given by control
    // scheme

    driveCommandDataAutoLogged.fieldRelative = fieldRelative;
    driveCommandDataAutoLogged.xSpeed = xSpeed;
    driveCommandDataAutoLogged.ySpeed = ySpeed;
    driveCommandDataAutoLogged.rot = rot;
    Logger.processInputs("Drive/Input", driveCommandDataAutoLogged);

    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rot,
                RobotContainer.poseTracker.getEstimatedPosition().getRotation())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);

    // Desaturates wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.robotMaxSpeed);

    // Sets state for each module
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    Logger.recordOutput("SwerveStates/Setpoints", swerveModuleStates);
  }

  public void SysIdOnlyDriveMotors(Voltage volts) {
    m_frontLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
    m_frontRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
    m_backLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
    m_backRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
  }

  public double SysIdOnlyGetDriveMotorVolts() {
    // average of all drive motors .get() values
    return (m_frontLeft.getDriveMotorVoltsSysIdOnly()
            + m_frontRight.getDriveMotorVoltsSysIdOnly()
            + m_backLeft.getDriveMotorVoltsSysIdOnly()
            + m_backRight.getDriveMotorVoltsSysIdOnly())
        / 4;
  }

  public double SysIdOnlyGetDriveMotorPosition() {
    // average
    return (m_frontLeft.getDriveWheelDistance()
            + m_frontRight.getDriveWheelDistance()
            + m_backLeft.getDriveWheelDistance()
            + m_backRight.getDriveWheelDistance())
        / 4;
  }

  public double SysIdOnlyGetDriveMotorVelocity() {
    // average
    return (m_frontLeft.getDriveWheelVelocity()
            + m_frontRight.getDriveWheelVelocity()
            + m_backLeft.getDriveWheelVelocity()
            + m_backRight.getDriveWheelVelocity())
        / 4;
  }

  public void SysIdOnlyTurnMotors(Voltage volts) {
    m_frontLeft.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
    m_frontRight.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
    m_backLeft.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
    m_backRight.setTurnMotorVoltsSysIdOnly(volts.in(Volts));
  }

  public double SysIdOnlyGetTurnMotorVolts() {
    // average of all turn motors .get() values
    return (m_frontLeft.getTurnMotorVoltsSysIdOnly()
            + m_frontRight.getTurnMotorVoltsSysIdOnly()
            + m_backLeft.getTurnMotorVoltsSysIdOnly()
            + m_backRight.getTurnMotorVoltsSysIdOnly())
        / 4;
  }

  public double SysIdOnlyGetTurnMotorPosition() {
    // average
    return (m_frontLeft.getTurnWheelRotation2d().getRotations()
            + m_frontRight.getTurnWheelRotation2d().getRotations()
            + m_backLeft.getTurnWheelRotation2d().getRotations()
            + m_backRight.getTurnWheelRotation2d().getRotations())
        / 4;
  }

  public double SysIdOnlyGetTurnMotorVelocity() {
    // average
    return (m_frontLeft.getTurnWheelVelocity()
            + m_frontRight.getTurnWheelVelocity()
            + m_backLeft.getTurnWheelVelocity()
            + m_backRight.getTurnWheelVelocity())
        / 4;
  }

  /**
   * Sets the PID target to zero and immediately stops all swerve modules.
   *
   * <p>This method commands the drivetrain to stop by setting the drive speeds to zero, thus
   * ensuring that the robot comes to a halt. It also directly stops each swerve module by setting
   * their motor outputs to zero.
   */
  @Override
  public void kill() {
    drive(new DriveCommandData(0, 0, 0, false));
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  /**
   * Retrieves the current chassis speeds relative to the robot's orientation.
   *
   * <p>This method calculates the chassis speeds based on the current states of all four swerve
   * modules using the drivetrain's kinematics.
   *
   * @return The current relative chassis speeds as a ChassisSpeeds object.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        m_frontLeft.getModuleState(),
        m_frontRight.getModuleState(),
        m_backLeft.getModuleState(),
        m_backRight.getModuleState());
  }

  /**
   * Returns the swerve drive kinematics for this drivetrain.
   *
   * @return The SwerveDriveKinematics object associated with this drivetrain.
   */
  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getModulePosition(),
      m_frontRight.getModulePosition(),
      m_backLeft.getModulePosition(),
      m_backRight.getModulePosition()
    };
  }

  public Command sysIdQuasistaticDriveMotors(SysIdRoutine.Direction direction) {
    return sysIdRoutineDriveMotors
        .quasistatic(direction)
        .beforeStarting(
            new WaitCommand(0.5)
                .deadlineFor(run(() -> drive(new DriveCommandData(0, 0, 0, false)))));
    // run pids with zero velocity for 0.5 seconds in order to align wheels;
  }

  public Command sysIdDynamicDriveMotors(SysIdRoutine.Direction direction) {
    return sysIdRoutineDriveMotors
        .dynamic(direction)
        .beforeStarting(
            new WaitCommand(0.5)
                .deadlineFor(run(() -> drive(new DriveCommandData(0, 0, 0, false)))));
    // run pids with zero velocity for 0.5 seconds in order to align wheels;
  }

  public Command sysIdQuasistaticTurnMotors(SysIdRoutine.Direction direction) {
    return sysIdRoutineTurnMotors
        .quasistatic(direction)
        .beforeStarting(
            new WaitCommand(0.5)
                .deadlineFor(run(() -> drive(new DriveCommandData(0, 0, 0, false)))));
    // run pids with zero velocity for 0.5 seconds in order to align wheels;
  }

  public Command sysIdDynamicTurnMotors(SysIdRoutine.Direction direction) {
    return sysIdRoutineTurnMotors
        .dynamic(direction)
        .beforeStarting(
            new WaitCommand(0.5)
                .deadlineFor(run(() -> drive(new DriveCommandData(0, 0, 0, false)))));
    // run pids with zero velocity for 0.5 seconds in order to align wheels;
  }

  /** frees up all hardware allocations */
  public void close() {
    m_backLeft.close();
    m_backRight.close();
    m_frontLeft.close();
    m_frontRight.close();
  }

  @Override
  public void setupShuffleboard() {
    SwerveModule[] modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
    for (SwerveModule module : modules) {
      module.setupShuffleboard();
    }
  }
}
