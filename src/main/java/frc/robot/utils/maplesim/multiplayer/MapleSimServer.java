package frc.robot.utils.maplesim.multiplayer;

import com.jfastnet.IServerHooks;
import com.jfastnet.Server;
import frc.robot.utils.maplesim.multiplayer.messages.context.MapleSimContext;

public class MapleSimServer {

    private final Server server;

    public MapleSimServer() {
        MapleSimContext context = MapleSimContext.createServerContext();
        server = new Server(Constants.createGeneralConfig(context)
                .setHost("0.0.0.0")
                .setBindPort(Constants.SERVER_PORT)
                .setServerHooks(new IServerHooks() {
                    /** Called when a client joins the server. */
                    @Override
                    public void onRegister(int clientId) {
                        System.out.println("Client connected: " + clientId);
                    }

                    /** Called when a client leaves the server. */
                    @Override
                    public void onUnregister(int clientId) {
                        System.out.println("Client disconnected: " + clientId);
                    }
                }));
        server.start();
    }

    public void stop() {
        server.stop();
    }
}
