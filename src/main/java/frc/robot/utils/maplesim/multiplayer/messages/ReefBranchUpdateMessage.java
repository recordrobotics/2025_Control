package frc.robot.utils.maplesim.multiplayer.messages;

import com.jfastnet.messages.Message;
import frc.robot.utils.maplesim.multiplayer.MapleSimClient.ReefBranchUpdateSetItem;
import frc.robot.utils.maplesim.multiplayer.messages.context.MapleSimContext;
import java.io.Serializable;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

public class ReefBranchUpdateMessage extends Message<MapleSimContext> {

    private ReefBranchUpdate[] updates;

    public ReefBranchUpdateMessage() {}

    public ReefBranchUpdateMessage(ReefBranchUpdateSetItem[] items) {
        updates = new ReefBranchUpdate[items.length];
        Arena2025Reefscape arena = (Arena2025Reefscape) SimulatedArena.getInstance();
        for (int i = 0; i < items.length; i++) {
            updates[i] = new ReefBranchUpdate(
                    items[i].isBlue(),
                    items[i].id(),
                    (items[i].isBlue() ? arena.blueReefSimulation : arena.redReefSimulation)
                            .getBranch(items[i].id())
                            .getGamePieceCount());
        }
    }

    @Override
    public void process(MapleSimContext context) {
        if (!context.isServer()) {
            for (ReefBranchUpdate update : updates) {
                System.out.println("Received ReefBranchUpdateMessage: isBlue=" + update.isBlue + ", id=" + update.id
                        + ", gamePieceCount=" + update.gamePieceCount);
            }
        }
    }

    @Override
    public ReliableMode getReliableMode() {
        return ReliableMode.SEQUENCE_NUMBER;
    }

    @Override
    public boolean stackable() {
        return true;
    }

    @Override
    public boolean broadcast() {
        return true;
    }

    @Override
    public boolean sendBroadcastBackToSender() {
        return false;
    }

    public record ReefBranchUpdate(boolean isBlue, String id, int gamePieceCount) implements Serializable {}

    static final long serialVersionUID = 1L;
}
