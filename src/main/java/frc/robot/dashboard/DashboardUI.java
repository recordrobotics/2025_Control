package frc.robot.dashboard;

public final class DashboardUI {
    private DashboardUI() {}

    public static final AutonomousLayout Autonomous = new AutonomousLayout();
    public static final OverviewLayout Overview = new OverviewLayout();

    public static void update() {
        Autonomous.update();
        Overview.update();
    }
}
