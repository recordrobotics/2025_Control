package frc.robot.utils.maplesim.multiplayer;

import com.jfastnet.Client;
import com.jfastnet.Config;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.maplesim.multiplayer.messages.RobotStateUpdateMessage;

public class MapleSimClient {

    private final Client client;

    public MapleSimClient(String host) {
        client = new Client(new Config().setHost(host).setPort(Constants.SERVER_PORT));
        client.start();
    }

    public void waitForConnection() {
        client.blockingWaitUntilConnected();
    }

    public void sendRobotStateUpdate(Pose2d pose) {
        if (!client.send(new RobotStateUpdateMessage(pose))) {
            ConsoleLogger.logError("Failed to send RobotStateUpdateMessage");
        }
    }
}
