package frc.robot.dashboard;

public class DashboardUI {
  public static final AutonomousLayout Autonomous = new AutonomousLayout();
  public static final OverviewLayout Overview = new OverviewLayout();
  public static final TestLayout Test = new TestLayout();

  public static void update() {
    Autonomous.update();
    Overview.update();
  }
}
