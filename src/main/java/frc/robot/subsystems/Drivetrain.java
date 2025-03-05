package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.RobotState.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.real.SwerveModuleReal;
import frc.robot.subsystems.io.sim.SwerveModuleSim;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.DriveCommandDataAutoLogged;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.ShuffleboardPublisher;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends KillableSubsystem
    implements ShuffleboardPublisher, PoweredSubsystem {
  // Creates swerve module objects
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private DriveCommandDataAutoLogged driveCommandDataAutoLogged = new DriveCommandDataAutoLogged();

  private final SysIdRoutine sysIdRoutineDriveMotors;
  private final SysIdRoutine sysIdRoutineTurnMotors;

  // Creates swerve kinematics
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          Constants.Swerve.frontLeftConstants.wheelLocation,
          Constants.Swerve.frontRightConstants.wheelLocation,
          Constants.Swerve.backLeftConstants.wheelLocation,
          Constants.Swerve.backRightConstants.wheelLocation);

  public Drivetrain() {
    if (Constants.RobotState.getMode() == Mode.REAL) {
      m_frontLeft =
          new SwerveModule(
              Constants.Swerve.frontLeftConstants,
              new SwerveModuleReal(Constants.Swerve.kDt, Constants.Swerve.frontLeftConstants));
      m_frontRight =
          new SwerveModule(
              Constants.Swerve.frontRightConstants,
              new SwerveModuleReal(Constants.Swerve.kDt, Constants.Swerve.frontRightConstants));
      m_backLeft =
          new SwerveModule(
              Constants.Swerve.backLeftConstants,
              new SwerveModuleReal(Constants.Swerve.kDt, Constants.Swerve.backLeftConstants));
      m_backRight =
          new SwerveModule(
              Constants.Swerve.backRightConstants,
              new SwerveModuleReal(Constants.Swerve.kDt, Constants.Swerve.backRightConstants));
    } else {
      m_frontLeft =
          new SwerveModule(
              Constants.Swerve.frontLeftConstants,
              new SwerveModuleSim(Constants.Swerve.kDt, Constants.Swerve.frontLeftConstants));
      m_frontRight =
          new SwerveModule(
              Constants.Swerve.frontRightConstants,
              new SwerveModuleSim(Constants.Swerve.kDt, Constants.Swerve.frontRightConstants));
      m_backLeft =
          new SwerveModule(
              Constants.Swerve.backLeftConstants,
              new SwerveModuleSim(Constants.Swerve.kDt, Constants.Swerve.backLeftConstants));
      m_backRight =
          new SwerveModule(
              Constants.Swerve.backRightConstants,
              new SwerveModuleSim(Constants.Swerve.kDt, Constants.Swerve.backRightConstants));
    }

    sysIdRoutineDriveMotors =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                Volts.of(2.5).per(Second),
                Volts.of(2.2),
                Seconds.of(1.0),
                (state ->
                    Logger.recordOutput("Drivetrain/Drive/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(this::SysIdOnlyDriveMotors, null, this));

    sysIdRoutineTurnMotors =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(
                Volts.of(6).per(Second),
                Volts.of(7),
                Seconds.of(1.5),
                (state -> Logger.recordOutput("Drivetrain/Turn/SysIdTestState", state.toString()))),
            new SysIdRoutine.Mechanism(this::SysIdOnlyTurnMotors, null, this));
  }

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

  @Override
  public void simulationPeriodic() {
    m_frontLeft.simulationPeriodic();
    m_frontRight.simulationPeriodic();
    m_backLeft.simulationPeriodic();
    m_backRight.simulationPeriodic();
  }

  public void SysIdOnlyDriveMotors(Voltage volts) {
    m_frontLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
    m_frontRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
    m_backLeft.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
    m_backRight.setDriveMotorVoltsSysIdOnly(volts.in(Volts));
  }

  @AutoLogOutput
  public double SysIdOnlyGetDriveMotorVolts() {
    // average of all drive motors .get() values
    return (m_frontLeft.getDriveMotorVoltsSysIdOnly()
            + m_frontRight.getDriveMotorVoltsSysIdOnly()
            + m_backLeft.getDriveMotorVoltsSysIdOnly()
            + m_backRight.getDriveMotorVoltsSysIdOnly())
        / 4;
  }

  @AutoLogOutput
  public double SysIdOnlyGetDriveMotorPosition() {
    // average
    return (m_frontLeft.getDriveWheelDistance()
            + m_frontRight.getDriveWheelDistance()
            + m_backLeft.getDriveWheelDistance()
            + m_backRight.getDriveWheelDistance())
        / 4;
  }

  @AutoLogOutput
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

  @AutoLogOutput
  public double SysIdOnlyGetTurnMotorVolts() {
    // average of all turn motors .get() values
    return (m_frontLeft.getTurnMotorVoltsSysIdOnly()
            + m_frontRight.getTurnMotorVoltsSysIdOnly()
            + m_backLeft.getTurnMotorVoltsSysIdOnly()
            + m_backRight.getTurnMotorVoltsSysIdOnly())
        / 4;
  }

  @AutoLogOutput
  public double SysIdOnlyGetTurnMotorPosition() {
    // average
    return (m_frontLeft.getTurnWheelRotation2d().getRotations()
            + m_frontRight.getTurnWheelRotation2d().getRotations()
            + m_backLeft.getTurnWheelRotation2d().getRotations()
            + m_backRight.getTurnWheelRotation2d().getRotations())
        / 4;
  }

  @AutoLogOutput
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
  @AutoLogOutput
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
        .raceWith(Commands.run(() -> drive(new DriveCommandData())));
    // run pids with zero velocity for 0.5 seconds in order to align wheels;
  }

  public Command sysIdDynamicDriveMotors(SysIdRoutine.Direction direction) {
    return sysIdRoutineDriveMotors
        .dynamic(direction)
        .raceWith(Commands.run(() -> drive(new DriveCommandData())));
    // run pids with zero velocity for 0.5 seconds in order to align wheels;
  }

  public Command sysIdQuasistaticTurnMotors(SysIdRoutine.Direction direction) {
    return sysIdRoutineTurnMotors.quasistatic(direction);
  }

  public Command sysIdDynamicTurnMotors(SysIdRoutine.Direction direction) {
    return sysIdRoutineTurnMotors.dynamic(direction);
  }

  /** frees up all hardware allocations */
  public void close() throws Exception {
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

  @Override
  public double getCurrentDrawAmps() {
    return m_frontLeft.getCurrentDrawAmps()
        + m_frontRight.getCurrentDrawAmps()
        + m_backLeft.getCurrentDrawAmps()
        + m_backRight.getCurrentDrawAmps();
  }
}
