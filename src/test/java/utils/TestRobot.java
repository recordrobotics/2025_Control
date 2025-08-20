package utils;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
import frc.robot.tests.TestControlBridge;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class TestRobot {

    static {
        if (!Constants.RobotState.UNIT_TESTS_ENABLE_ADVANTAGE_SCOPE) {
            NetworkTableInstance.getDefault().startLocal(); // disable nt
        }
        Unmanaged.setPhoenixDiagnosticsStartTime(-1); // disable phoenix
        Constants.RobotState.runningAsUnitTest = true;
        RobotController.setTimeSource(TestRobot::getTimestamp);
    }

    private static volatile Robot testRobot;
    private static Thread robotThread;
    private static final ReentrantLock m_runMutex = new ReentrantLock();
    private static final List<Supplier<Boolean>> m_periodicRunnables = new ArrayList<>();
    private static final List<Supplier<Boolean>> m_periodicRunnablesToRemove = new ArrayList<>();
    private static final List<Consumer<Command>> m_commandFinishListeners = new ArrayList<>();

    private static long timestamp = 0;
    private static int periodicCount = 0;

    /**
     * Acts as the robot time source
     */
    private static long getTimestamp() {
        return timestamp;
    }

    /**
     * Local callback to run periodic.
     * This is called every 20ms (50Hz) by the robot code.
     * It runs all the periodic runnables that were added to the list.
     * Also increments the timestamp by 20ms to simulate the robot time.
     */
    private static void periodicRunnable() {
        timestamp += 20 * 1000; // 20ms in microseconds
        periodicCount++;

        m_runMutex.lock();

        for (Supplier<Boolean> r : m_periodicRunnables) {
            if (!r.get()) {
                // If the runnable returns false, it is marked for removal
                m_periodicRunnablesToRemove.add(r);
            }
        }

        // Remove periodic runnables that were marked for removal
        m_periodicRunnables.removeAll(m_periodicRunnablesToRemove);
        m_periodicRunnablesToRemove.clear();

        m_runMutex.unlock();
    }

    /**
     * Local callback to handle command finishes
     * @param cmd the command that just finished
     */
    private static void onCommandFinish(Command cmd) {
        m_runMutex.lock();

        for (Consumer<Command> c : m_commandFinishListeners) {
            c.accept(cmd);
        }

        m_runMutex.unlock();
    }

    /**
     * Ends the robot and cleans up resources.
     * This should only be called after all tests are done running,
     * since the unit tests reuse the robot instance to save time
     */
    public static void endRobot() {
        if (testRobot != null) {
            testRobot.endCompetition();

            try {
                robotThread.join(5000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            testRobot = null;
        }
    }

    /**
     * Creates a stop condition that will return true when the command ends that matches the given predicate.
     * @param commandPredicate a predicate that tests the command to determine if it should stop
     * @return a supplier that returns true when the command ends that matches the predicate
     */
    public static Supplier<Boolean> stopOnCommandEnd(Predicate<Command> commandPredicate) {
        boolean[] shouldStop = {false};
        Consumer<Command> listener = cmd -> {
            if (commandPredicate.test(cmd)) {
                shouldStop[0] = true;
            }
        };
        m_runMutex.lock();
        m_commandFinishListeners.add(listener);
        m_runMutex.unlock();

        return () -> {
            boolean stop = shouldStop[0];
            if (stop) {
                m_runMutex.lock();
                m_commandFinishListeners.remove(listener);
                m_runMutex.unlock();
            }
            return stop;
        };
    }

    /**
     * Creates a stop condition that will return true after a certain number of periodic cycles
     * when the stop condition is met. If the stop condition returns false after being true, it resets the cycle count.
     * @param cycles the number of periodic cycles to wait before stopping, if less than or equal to 0, it will stop immediately
     * @param stopCondition a supplier that returns true when the stop condition is met
     * @return a supplier that returns true when the stop condition is met after the specified cycles
     */
    public static Supplier<Boolean> delayStop(int cycles, Supplier<Boolean> stopCondition) {
        if (stopCondition == null) throw new IllegalArgumentException("stopCondition cannot be null");

        if (cycles <= 0) {
            return stopCondition; // If cycles is less than or equal to 0, return the stop condition directly
        }

        int[] currentCount = {-1};
        return () -> {
            boolean stop = stopCondition.get();
            if (stop) {
                if (currentCount[0] == -1) {
                    currentCount[0] = periodicCount;
                } else if (periodicCount >= currentCount[0] + cycles) {
                    return true; // Stop condition met after the specified cycles
                }
            } else {
                currentCount[0] = -1; // Reset if stop condition is not met
            }
            return false;
        };
    }

    /**
     * Runs the robot until the stop condition is met, with a robot configuration and optional periodic runnable.
     * Finally, runs the test function to verify the robot's state.
     * The default timeout is 10 seconds.
     * @param stopCondition a supplier that returns true when the test should stop
     * @param periodic an optional periodic runnable that runs every 20ms (can also be null). return true to call it next periodic, false to stop calling it.
     * @param robotConfig a consumer that configures the robot before the test starts
     * @param testFunction a runnable that runs after the stop condition is met to verify the robot's state
     */
    public static void testUntil(
            Supplier<Boolean> stopCondition,
            Optional<Supplier<Boolean>> periodic,
            Consumer<Robot> robotConfig,
            Runnable testFunction) {
        testUntil(stopCondition, periodic, robotConfig, testFunction, 10.0);
    }

    /**
     * Runs the robot until the stop condition is met, with a robot configuration and optional periodic runnable.
     * Finally, runs the test function to verify the robot's state.
     * @param stopCondition a supplier that returns true when the test should stop
     * @param periodic an optional periodic boolean supplier that runs every 20ms (can also be null). return true to call it next periodic, false to stop calling it.
     * @param robotConfig a consumer that configures the robot before the test starts
     * @param testFunction a runnable that runs after the stop condition is met to verify the robot's state
     * @param timeoutSeconds the maximum time to run the test before failing (in seconds), if less than or equal to 0, no timeout
     */
    public static void testUntil(
            Supplier<Boolean> stopCondition,
            Optional<Supplier<Boolean>> periodic,
            Consumer<Robot> robotConfig,
            Runnable testFunction,
            double timeoutSeconds) {

        if (stopCondition == null) throw new IllegalArgumentException("stopCondition cannot be null");
        if (robotConfig == null) throw new IllegalArgumentException("robotConfig cannot be null");
        if (testFunction == null) throw new IllegalArgumentException("testFunction cannot be null");

        if (testRobot == null) {
            m_runMutex.lock();
            testRobot = new Robot();

            RobotController.setTimeSource(TestRobot::getTimestamp);
            testRobot.setPeriodicRunnable(TestRobot::periodicRunnable);
            CommandScheduler.getInstance().onCommandFinish(TestRobot::onCommandFinish);

            robotThread = new Thread(() -> {
                Constants.RobotState.runningAsUnitTest = true;
                RobotBase.startRobot(() -> testRobot);
            });
            robotThread.setDaemon(true);
            robotThread.setName("TestRobot Daemon");
            robotThread.start();
            m_runMutex.unlock();
        }

        m_runMutex.lock();
        TestRobot.waitForInit();
        robotConfig.accept(testRobot);
        if (periodic != null && periodic.isPresent()) {
            m_periodicRunnables.add(periodic.get());
        }
        m_runMutex.unlock();

        long startTime = getTimestamp();
        if (timeoutSeconds > 0) {
            Supplier<Boolean> originalStopCondition = stopCondition;
            stopCondition =
                    () -> originalStopCondition.get() || (getTimestamp() - startTime) >= (timeoutSeconds * 1000000);
        }

        try {
            while (!stopCondition.get()) {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        } finally {
            if (periodic != null && periodic.isPresent()) {
                m_runMutex.lock();
                m_periodicRunnables.remove(periodic.get());
                m_runMutex.unlock();
            }
        }

        testFunction.run();
    }

    /**
     * Runs a runnable after a certain number of periodic cycles.
     * @param cycles the number of periodic cycles to wait before running the runnable
     * @param runnable the runnable to run after the specified number of cycles
     * @throws IllegalArgumentException if cycles is less than or equal to 0, or if runnable is null
     */
    public static void runAfter(int cycles, Runnable runnable) {
        if (runnable == null) throw new IllegalArgumentException("runnable cannot be null");
        if (cycles <= 0) throw new IllegalArgumentException("cycles must be greater than 0");

        m_runMutex.lock();
        int currentPeriodicCount = periodicCount;
        m_periodicRunnables.add(() -> {
            if (currentPeriodicCount + cycles == periodicCount) {
                runnable.run();
                return false;
            }

            return true;
        });
        m_runMutex.unlock();
    }

    /**
     * Runs a runnable for a certain number of periodic cycles.
     * @param cycles the number of periodic cycles to run the runnable for
     * @param runnable the runnable to run for the specified number of cycles
     * @throws IllegalArgumentException if cycles is less than or equal to 0, or if runnable is null
     */
    public static void runFor(int cycles, Runnable runnable) {
        if (runnable == null) throw new IllegalArgumentException("runnable cannot be null");
        if (cycles <= 0) throw new IllegalArgumentException("cycles must be greater than 0");

        m_runMutex.lock();
        int currentPeriodicCount = periodicCount;
        m_periodicRunnables.add(() -> {
            if (periodicCount <= currentPeriodicCount + cycles) {
                runnable.run();
                return true;
            }

            return false;
        });
        m_runMutex.unlock();
    }

    /**
     * Resets important robot states and waits for the robot to be initialized.
     * Runs before every unit test (call to robotConfig)
     */
    public static void waitForInit() {
        TestControlBridge.getInstance().reset();
        SimulatedArena.getInstance().clearGamePieces();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        while (!testRobot.isInitialized()) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        try {
            RobotContainer.elevatorHead.getSimIO().clearPreload();
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            RobotContainer.coralIntake.getSimIO().removeCoral();
        } catch (Exception e) {
            e.printStackTrace();
        }

        testRobot.getRobotContainer().resetEncoders();
        CommandScheduler.getInstance().cancelAll();
        RobotContainer.elevator.set(Constants.Elevator.STARTING_HEIGHT);
        RobotContainer.elevatorArm.set(ElevatorHeight.BOTTOM.getArmAngle());
        RobotContainer.elevatorHead.set(CoralShooterStates.OFF);
        RobotContainer.coralIntake.set(CoralIntakeState.UP);
    }

    /**
     * Helper function for shorter access to {@link TestControlBridge#getInstance() TestControlBridge.getInstance()}
     */
    public static TestControlBridge controlBridge() {
        return TestControlBridge.getInstance();
    }
}
