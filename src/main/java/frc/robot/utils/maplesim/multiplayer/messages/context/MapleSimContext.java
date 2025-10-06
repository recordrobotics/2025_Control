package frc.robot.utils.maplesim.multiplayer.messages.context;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.HashMap;

public final class MapleSimContext {

    private final boolean server;
    private final HashMap<Integer, Pose3d> robotCorals;

    private MapleSimContext(boolean server) {
        this.server = server;
        if (server) {
            robotCorals = null;
        } else {
            robotCorals = new HashMap<>();
        }
    }

    public static MapleSimContext createClientContext() {
        return new MapleSimContext(false);
    }

    public boolean isServer() {
        return server;
    }

    public void addRobotCoral(int id, Pose3d pose) {
        if (!server) {
            robotCorals.put(id, pose);
        }
    }

    public static MapleSimContext createServerContext() {
        return new MapleSimContext(true);
    }

    public Pose3d[] getRobotCorals() {
        if (server) {
            throw new IllegalStateException("Cannot get robot corals from server context");
        }
        return robotCorals.values().stream().filter(p -> p != null).toArray(Pose3d[]::new);
    }
}
