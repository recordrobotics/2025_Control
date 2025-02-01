package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.DriverStationUtils;

public abstract class AbstractControl {
  // Movement
  public abstract DriveCommandData getDriveCommandData();

  // Misc
  public abstract Boolean getPoseReset();

  public abstract Boolean getKill();

  // Shoot coral
  public abstract Boolean getCoralShootL1();

  public abstract Boolean getCoralShootL2();

  public abstract Boolean getCoralShootL3();

  public abstract Boolean getCoralShootL4();

  // Intake coral
  public abstract Boolean getCoralGroundIntake();

  public abstract Boolean getCoralSourceIntake();

  // Intake algae
  public abstract Boolean getIntakeAlgae();

  // Reef algae
  public abstract Boolean getReefAlgae();

  // Score algae
  public abstract Boolean getScoreAlgae();

  // Climb
  public abstract Boolean getClimb();

  public abstract void vibrate(double value);

  // Orient XY
  public static Pair<Double, Double> OrientXY(Pair<Double, Double> input) {
    double inputX = input.getFirst();
    double inputY = input.getSecond();

    switch (DashboardUI.Overview.getDriverOrientation()) {
      case XAxisTowardsTrigger:
        if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
          return new Pair<Double, Double>(-inputY, -inputX);
        else return new Pair<Double, Double>(inputY, inputX);
      case YAxisTowardsTrigger:
        if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
          return new Pair<Double, Double>(inputX, -inputY);
        else return new Pair<Double, Double>(-inputX, inputY);
      case XAxisInvertTowardsTrigger:
        if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
          return new Pair<Double, Double>(inputY, inputX);
        else return new Pair<Double, Double>(-inputY, -inputX);
      default:
        return new Pair<Double, Double>(0.0, 0.0);
    }
  }

  // Orient Angle
  public static Rotation2d OrientAngle(Rotation2d angle) {
    switch (DashboardUI.Overview.getDriverOrientation()) {
      case XAxisTowardsTrigger:
        if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
          return new Rotation2d(angle.getRadians() - Math.PI / 2);
        else return new Rotation2d(angle.getRadians() + Math.PI / 2);
      case YAxisTowardsTrigger:
        if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) return angle;
        else return new Rotation2d(angle.getRadians() + Math.PI);
      default:
        return angle;
    }
  }
}
