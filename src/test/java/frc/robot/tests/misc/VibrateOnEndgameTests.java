package frc.robot.tests.misc;

import static edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setFmsAttached;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setMatchTime;
import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static utils.TestRobot.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.commands.VibrateXbox;
import org.junit.jupiter.api.Test;

class VibrateOnEndgameTests {

    @Test
    void testVibrateWithFMS() {
        testUntil(
                stopOnCommandInit(VibrateXbox.class::isInstance),
                () -> {
                    return true;
                },
                robot -> {
                    System.out.println("Setting up FMS");
                    setAllianceStationId(AllianceStationID.Blue1);
                    setFmsAttached(true);
                    setMatchTime(30);
                    setEnabled(true);
                    notifyNewData();
                },
                () -> {
                    assertTrue(controlBridge().getRumble(RumbleType.kBothRumble) > 0, "Controller should be vibrating");
                });
    }

    @Test
    void testNoVibrateInAutoWithFMS() {}

    @Test
    void testVibrateWithoutFMS() {}

    @Test
    void testNoVibrateInAutoWithoutFMS() {}
}
