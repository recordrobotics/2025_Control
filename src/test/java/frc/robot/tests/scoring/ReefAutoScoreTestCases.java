package frc.robot.tests.scoring;

import static edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled;
import static frc.robot.tests.TestControlBridge.Button.*;
import static org.junit.jupiter.api.DynamicTest.dynamicTest;
import static utils.Assertions.assertReefEquals;
import static utils.TestRobot.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.Game.*;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoScore;
import frc.robot.control.AbstractControl.ReefLevelSwitchValue;
import java.util.Arrays;
import java.util.stream.Stream;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;

public class ReefAutoScoreTestCases {

    private static Stream<DynamicTest> testFactory(Stream<CoralPosition> branches, int level) {
        return branches.map(coralPosition -> {
            // Branch id is the first and last letter of CoralPosition + the level (BlueA, 4 -> BA4)
            String branchId = String.valueOf(coralPosition.name().charAt(0))
                    + String.valueOf(
                            coralPosition.name().charAt(coralPosition.name().length() - 1))
                    + level;

            return dynamicTest("AutoScore " + branchId, () -> {
                testUntil(
                        delayStop(level == 1 ? 20 : 0, stopOnCommandEnd(AutoScore.class::isInstance)),
                        null,
                        robot -> {
                            /* robot and field setup */

                            final Pose2d startPose =
                                    coralPosition.getPose().transformBy(new Transform2d(-2.0, 0, Rotation2d.kZero));
                            RobotContainer.drivetrain.getSwerveDriveSimulation().setSimulationWorldPose(startPose);

                            // Odometry reset has to run during periodic to work correctly
                            runFor(2, () -> RobotContainer.poseSensorFusion.setToPose(startPose));

                            controlBridge().setReefLevel(ReefLevelSwitchValue.valueOf("L" + level));

                            setAllianceStationId(AllianceStationID.Blue1);
                            setEnabled(true);
                            notifyNewData();

                            if (level != 1) {
                                // Give robot preload
                                try {
                                    RobotContainer.elevatorHead.getSimIO().setPreload();
                                } catch (Exception e) {
                                    e.printStackTrace();
                                }
                            } else {
                                // Give robot preload
                                try {
                                    RobotContainer.coralIntake.getSimIO().addCoral();
                                } catch (Exception e) {
                                    e.printStackTrace();
                                }
                            }

                            // Because of debouncer on coral sensor - have to wait two periodic cycles before autoscore
                            runAfter(2, () -> controlBridge().pressButton(AUTO_SCORE, 1));
                        },
                        () -> assertReefEquals(branchId));
            });
        });
    }

    static class Blue1 {
        @TestFactory
        Stream<DynamicTest> blue1() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("Blue")), 1);
        }
    }

    static class Blue2 {
        @TestFactory
        Stream<DynamicTest> blue2() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("Blue")), 2);
        }
    }

    static class Blue3 {
        @TestFactory
        Stream<DynamicTest> blue3() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("Blue")), 3);
        }
    }

    static class Blue4 {
        @TestFactory
        Stream<DynamicTest> blue4() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("Blue")), 4);
        }
    }

    static class Red1 {
        @TestFactory
        Stream<DynamicTest> red1() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("Red")), 1);
        }
    }

    static class Red2 {
        @TestFactory
        Stream<DynamicTest> red2() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("Red")), 2);
        }
    }

    static class Red3 {
        @TestFactory
        Stream<DynamicTest> red3() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("Red")), 3);
        }
    }

    static class Red4 {
        @TestFactory
        Stream<DynamicTest> red4() {
            return testFactory(
                    Arrays.stream(CoralPosition.values()).filter(c -> c.name().startsWith("Red")), 4);
        }
    }
}
