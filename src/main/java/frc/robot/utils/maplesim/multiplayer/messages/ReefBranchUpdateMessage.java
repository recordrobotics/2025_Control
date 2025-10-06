package frc.robot.utils.maplesim.multiplayer.messages;

import com.jfastnet.messages.Message;
import frc.robot.utils.maplesim.multiplayer.messages.context.MapleSimContext;

public class ReefBranchUpdateMessage extends Message<MapleSimContext> {

    private boolean isBlue;
    private String id;
    private int gamePieceCount;

    public ReefBranchUpdateMessage() {}

    public ReefBranchUpdateMessage(boolean isBlue, String id, int gamePieceCount) {
        this.isBlue = isBlue;
        this.id = id;
        this.gamePieceCount = gamePieceCount;
    }

    @Override
    public void process(MapleSimContext context) {
        if (!context.isServer()) {
            System.out.println("Received ReefBranchUpdateMessage: isBlue=" + isBlue + ", id=" + id + ", gamePieceCount="
                    + gamePieceCount);
        }
    }

    @Override
    public ReliableMode getReliableMode() {
        return ReliableMode.SEQUENCE_NUMBER;
    }

    @Override
    public boolean stackable() {
        return false;
    }

    @Override
    public boolean broadcast() {
        return true;
    }

    @Override
    public boolean sendBroadcastBackToSender() {
        return false;
    }

    static final long serialVersionUID = 1L;
}
