// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.KillSpecified;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.AutoLogLevelManager;
import frc.robot.utils.LocalADStarAK;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdRoutine;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private static double autoStartTimestamp;

  private static final String defaultPathRio = "/home/lvuser/logs";
  private static final String defaultPathSim = "logs";

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REPLAY) {
      setUseTiming(true); // Run at standard robot speed (20 ms)
      Logger.addDataReceiver(
          new WPILOGWriter(RobotBase.isSimulation() ? defaultPathSim : defaultPathRio));
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    DriverStation.silenceJoystickConnectionWarning(
        Constants.RobotState.getMode() == Constants.RobotState.Mode.SIM);

    Logger.start();

    if (Constants.RobotState.MOTOR_LOGGING_ENABLED) {
      for (int i = 0; i < 10; i++) {
        DriverStation.reportWarning(
            "[WARNING] Motor logging enabled, DON'T FORGET to delete old logs to make space on disk.\n"
                + "[WARNING] During competition, set MOTOR_LOGGING_ENABLED to false since logging is enabled automatically.",
            false);
      }
      SignalLogger.start();
    }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStarAK());
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Elastic layout webserver
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    if (Constants.RobotState.getMode() == Constants.RobotState.Mode.SIM) {
      // Reset simulation field
      SimulatedArena.getInstance().resetFieldForAuto();
    }

    // MAKE SURE FIRST CALL TO ELASTIC IS NOT IN TELEOP OR AUTO INIT!!
    DashboardUI.Autonomous.switchTo();

    AutoLogLevelManager.addObject(this);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    DashboardUI.Overview.getControl().update();

    // End and start reversed to make sure we get latest data before command scheduler
    RobotContainer.poseSensorFusion.endCalculation();
    RobotContainer.poseSensorFusion.startCalculation();

    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    // Threads.setCurrentThreadPriority(false, 10);

    DashboardUI.update();

    AutoLogLevelManager.periodic();
  }

  boolean hasRun = false;

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (SysIdManager.getSysIdRoutine() != SysIdRoutine.None && hasRun) {
      Logger.end();
      SignalLogger.stop();
    }

    m_robotContainer.disabledInit();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoStartTimestamp = Timer.getFPGATimestamp();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Cancel any previous commands
    CommandScheduler.getInstance().cancelAll();

    if (Constants.RobotState.getMode() == Constants.RobotState.Mode.SIM) {
      // Reset simulation field
      SimulatedArena.getInstance().resetFieldForAuto();
      RobotContainer.humanPlayerSimulation.reset();

      // Give robot preload
      try {
        RobotContainer.elevatorHead.getSimIO().setPreload();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    DashboardUI.Autonomous.switchTo();

    hasRun = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    new KillSpecified(RobotContainer.elevatorHead, RobotContainer.coralIntake).schedule();

    m_robotContainer.teleopInit();
    hasRun = true;

    DashboardUI.Overview.switchTo();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Logger.recordOutput(
        "Control/ReefLevelSwitch", DashboardUI.Overview.getControl().getReefLevelSwitchValue());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_robotContainer.testPeriodic();
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    m_robotContainer.simulationPeriodic();
  }

  public static double getAutoStartTime() {
    return autoStartTimestamp;
  }
}
