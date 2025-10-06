package frc.robot.utils.maplesim.multiplayer;

import com.esotericsoftware.kryo.Kryo;
import com.jfastnet.Config;
import com.jfastnet.IMessageReceiver;
import com.jfastnet.config.SerialiserConfig;
import com.jfastnet.idprovider.ReliableModeIdProvider;
import com.jfastnet.messages.AckMessage;
import com.jfastnet.messages.ClientTimerSyncMessage;
import com.jfastnet.messages.CompressedMessage;
import com.jfastnet.messages.ConnectRequest;
import com.jfastnet.messages.ConnectResponse;
import com.jfastnet.messages.GenericMessage;
import com.jfastnet.messages.IsReadyMessage;
import com.jfastnet.messages.LeaveConfirmationResponse;
import com.jfastnet.messages.LeaveRequest;
import com.jfastnet.messages.Message;
import com.jfastnet.messages.MessagePart;
import com.jfastnet.messages.RequestSeqIdsMessage;
import com.jfastnet.messages.SequenceKeepAlive;
import com.jfastnet.messages.StackAckMessage;
import com.jfastnet.messages.StackedMessage;
import com.jfastnet.messages.TimerSyncMessage;
import com.jfastnet.peers.netty.KryoNettyPeer;
import com.jfastnet.serialiser.KryoSerialiser;
import frc.robot.utils.maplesim.multiplayer.messages.ReefBranchUpdateMessage;
import frc.robot.utils.maplesim.multiplayer.messages.RobotStateUpdateMessage;
import frc.robot.utils.maplesim.multiplayer.messages.context.MapleSimContext;
import java.util.ArrayList;

public final class Constants {

    public static final int SERVER_PORT = 15150;

    private static final int KRYO_START_INDEX = 9;

    @SuppressWarnings("java:S1854")
    private static Kryo createKryo() {
        Kryo kryo = new Kryo();

        int index = KRYO_START_INDEX;

        kryo.register(double[].class, index++);
        kryo.register(ArrayList.class, index++);

        kryo.register(ConnectRequest.class, index++);
        kryo.register(ConnectResponse.class, index++);
        kryo.register(LeaveRequest.class, index++);
        kryo.register(AckMessage.class, index++);
        kryo.register(StackedMessage.class, index++);
        kryo.register(ClientTimerSyncMessage.class, index++);
        kryo.register(CompressedMessage.class, index++);
        kryo.register(GenericMessage.class, index++);
        kryo.register(IsReadyMessage.class, index++);
        kryo.register(LeaveConfirmationResponse.class, index++);
        kryo.register(Message.class, index++);
        kryo.register(MessagePart.class, index++);
        kryo.register(RequestSeqIdsMessage.class, index++);
        kryo.register(SequenceKeepAlive.class, index++);
        kryo.register(StackAckMessage.class, index++);
        kryo.register(TimerSyncMessage.class, index++);

        kryo.register(RobotStateUpdateMessage.class, index++);
        kryo.register(ReefBranchUpdateMessage.class, index++);

        return kryo;
    }

    @SuppressWarnings("java:S1604") // not generic
    public static Config createGeneralConfig(MapleSimContext context) {
        return new Config()
                .setContext(context)
                .setExternalReceiver(new IMessageReceiver<MapleSimContext>() {
                    @Override
                    public void receive(Message<MapleSimContext> message) {
                        message.process(context);
                    }
                })
                .setIdProviderClass(ReliableModeIdProvider.class)
                .setUdpPeerClass(KryoNettyPeer.class)
                .setSerialiser(
                        new KryoSerialiser(new SerialiserConfig(), ThreadLocal.withInitial(Constants::createKryo)));
    }

    private Constants() {}
}
