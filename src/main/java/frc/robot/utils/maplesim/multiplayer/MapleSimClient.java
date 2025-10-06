package frc.robot.utils.maplesim.multiplayer;

import com.jfastnet.Client;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.maplesim.multiplayer.messages.ReefBranchUpdateMessage;
import frc.robot.utils.maplesim.multiplayer.messages.RobotStateUpdateMessage;
import frc.robot.utils.maplesim.multiplayer.messages.context.MapleSimContext;
import java.util.Set;

public class MapleSimClient {

    private final Client client;
    private final Set<ReefBranchUpdateSetItem> reefBranchUpdateSet;

    public MapleSimClient(String host) {
        reefBranchUpdateSet = new java.util.concurrent.ConcurrentSkipListSet<>();
        MapleSimContext context = MapleSimContext.createClientContext();
        client = new Client(Constants.createGeneralConfig(context).setHost(host).setPort(Constants.SERVER_PORT));
        client.start();
    }

    public void waitForConnection() {
        client.blockingWaitUntilConnected();
    }

    public void sendRobotStateUpdate(Pose2d pose, Pose3d[] mechanismPoses, Pose3d robotCoral) {
        if (client.isConnected()) {
            if (!client.send(new RobotStateUpdateMessage(pose, mechanismPoses, robotCoral))) {
                ConsoleLogger.logError("Failed to send RobotStateUpdateMessage");
            }
        }
    }

    public void accumulateReefBranchUpdate(boolean isBlue, String id) {
        reefBranchUpdateSet.add(new ReefBranchUpdateSetItem(isBlue, id));
    }

    public void sendAccumulatedReefBranchUpdates() {
        if (client.isConnected()) {
            client.send(new ReefBranchUpdateMessage(reefBranchUpdateSet.toArray(ReefBranchUpdateSetItem[]::new)));
            reefBranchUpdateSet.clear();
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

    public record ReefBranchUpdateSetItem(boolean isBlue, String id) implements Comparable<ReefBranchUpdateSetItem> {

        @Override
        public int compareTo(ReefBranchUpdateSetItem o) {
            int blueComparison = Boolean.compare(this.isBlue, o.isBlue);
            if (blueComparison != 0) {
                return blueComparison;
            }
            return this.id.compareTo(o.id);
        }
    }
}
