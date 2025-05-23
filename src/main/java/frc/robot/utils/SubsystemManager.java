package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class SubsystemManager extends SubsystemBase {
  /** The Singleton Instance. */
  private static SubsystemManager instance;

  // A map from subsystems registered with the scheduler to their default commands.  Also used
  // as a list of currently-registered subsystems.
  private final Map<ManagedSubsystemBase, Command> m_subsystems = new LinkedHashMap<>();

  private final Watchdog m_watchdog = new Watchdog(TimedRobot.kDefaultPeriod, () -> {});

  private final ExecutorService executor = Executors.newCachedThreadPool();

  /**
   * Returns the Scheduler instance.
   *
   * @return the instance
   */
  public static synchronized SubsystemManager getInstance() {
    if (instance == null) {
      instance = new SubsystemManager();
    }
    return instance;
  }

  /**
   * Registers subsystems with the scheduler. This must be called for the subsystem's periodic block
   * to run when the scheduler is run, and for the subsystem's default command to be scheduled. It
   * is recommended to call this from the constructor of your subsystem implementations.
   *
   * @param subsystems the subsystem to register
   */
  public void registerSubsystem(ManagedSubsystemBase... subsystems) {
    for (ManagedSubsystemBase subsystem : subsystems) {
      if (subsystem == null) {
        DriverStation.reportWarning("Tried to register a null subsystem", true);
        continue;
      }
      if (m_subsystems.containsKey(subsystem)) {
        DriverStation.reportWarning("Tried to register an already-registered subsystem", true);
        continue;
      }
      m_subsystems.put(subsystem, null);
    }
  }

  private final List<Future<?>> m_futures = new ArrayList<>();

  @Override
  public void periodic() {
    m_watchdog.reset();
    m_futures.clear();

    // Run the periodic method of all registered subsystems.
    for (ManagedSubsystemBase subsystem : m_subsystems.keySet()) {
      m_futures.add(executor.submit(subsystem::periodicManaged));
      if (RobotBase.isSimulation()) {
        m_futures.add(executor.submit(subsystem::simulationPeriodicManaged));
      }
    }

    for (Future<?> future : m_futures) {
      try {
        future.get();
      } catch (Exception e) {
        DriverStation.reportError("SubsystemManager periodic error: " + e.getMessage(), false);
      }
    }

    m_watchdog.disable();
    if (m_watchdog.isExpired()) {
      System.out.println("SubsystemManager loop overrun: " + m_watchdog.getTime());
    }
  }
}
