package frc.robot.utils.maplesim.multiplayer;

import com.jfastnet.Client;
import com.jfastnet.Config;
import com.jfastnet.IMessageReceiver;
import com.jfastnet.messages.Message;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.maplesim.multiplayer.messages.RobotStateUpdateMessage;
import frc.robot.utils.maplesim.multiplayer.messages.context.MapleSimContext;

public class MapleSimClient {

    private final Client client;

    @SuppressWarnings("java:S1604") // not generic
    public MapleSimClient(String host) {
        MapleSimContext context = MapleSimContext.createClientContext();
        client = new Client(new Config()
                .setContext(context)
                .setExternalReceiver(new IMessageReceiver<MapleSimContext>() {
                    @Override
                    public void receive(Message<MapleSimContext> message) {
                        message.process(context);
                    }
                })
                .setHost(host)
                .setPort(Constants.SERVER_PORT));
        client.start();
    }

    public void waitForConnection() {
        client.blockingWaitUntilConnected();
    }

    public void sendRobotStateUpdate(Pose2d pose, Pose3d[] mechanismPoses, Pose3d robotCoral) {
        if (!client.send(new RobotStateUpdateMessage(pose, mechanismPoses, robotCoral))) {
            ConsoleLogger.logError("Failed to send RobotStateUpdateMessage");
        }
    }

    public Pose3d[] getRobotCorals() {
        MapleSimContext context = (MapleSimContext) client.getConfig().getContext();
        if (context == null) {
            ConsoleLogger.logError("MapleSimContext is null");
            return new Pose3d[] {};
        }
        return context.getRobotCorals();
    }
}
