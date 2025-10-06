package frc.robot.utils.maplesim.multiplayer;

import com.jfastnet.Config;
import com.jfastnet.IServerHooks;
import com.jfastnet.Server;

public class MapleSimServer {

    private final Server server;

    public MapleSimServer() {
        server = new Server(new Config()
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
