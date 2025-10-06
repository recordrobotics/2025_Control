package frc.robot.utils.maplesim.multiplayer.standalone.server;

import frc.robot.utils.maplesim.multiplayer.Constants;
import frc.robot.utils.maplesim.multiplayer.MapleSimServer;
import java.nio.charset.Charset;
import java.util.Scanner;

public final class Main {

    private Main() {}

    public static void main(String[] args) {
        org.apache.log4j.BasicConfigurator.configure();
        Constants.setServerMode(true);

        MapleSimServer server = new MapleSimServer();
        System.out.println("Started MapleSim server");

        while (true) {
            System.out.println("Type 'exit' to stop the server");
            try (Scanner scanner = new Scanner(System.in, Charset.defaultCharset())) {
                String line = scanner.nextLine();
                if (line.equalsIgnoreCase("exit")) {
                    break;
                }
            }
        }

        server.stop();
        System.out.println("Stopped MapleSim server");
        System.exit(0);
    }
}
