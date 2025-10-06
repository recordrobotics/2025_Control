package frc.robot.utils.maplesim.multiplayer;

public final class Constants {

    public static final int SERVER_PORT = 15150;

    private static boolean serverMode = false;

    public static boolean isInServerMode() {
        return serverMode;
    }

    public static void setServerMode(boolean isServer) {
        serverMode = isServer;
    }

    private Constants() {}
}
