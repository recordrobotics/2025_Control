package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.DriveCommandData;
import frc.robot.utils.DriverStationUtils;

public abstract class AbstractControl {
  // Movement
  public abstract DriveCommandData getDriveCommandData();

  public abstract Boolean getAutoAlign();

  public abstract Boolean getElevatorRelativeDrive();

  public abstract Boolean getCoralIntakeRelativeDrive();

  // Misc
  public abstract Boolean getPoseReset();

  public abstract Boolean getKill();

  // Elevator
  public abstract Boolean getAutoScore();

  public abstract Boolean getElevatorL2();

  public abstract Boolean getElevatorL3();

  public abstract Boolean getElevatorL4();

  public abstract Boolean getElevatorAlgaeLow();

  public abstract Boolean getElevatorAlgaeHigh();

  public abstract Boolean getManualOverride();

  public abstract LinearVelocity getManualElevatorVelocity();

  public abstract AngularVelocity getManualElevatorArmVelocity();

  // Intake coral
  public abstract Boolean getCoralGroundIntake();

  public abstract Boolean getCoralSourceIntake();

  public abstract Boolean getCoralIntakeScoreL1();

  public abstract Boolean getCoralShoot();

  // Ground Algae
  public abstract Boolean getGroundAlgae();

  // Score algae
  public abstract Boolean getScoreAlgae();

  public abstract Boolean getBargeAlgae();

  // Climb
  public abstract Boolean getClimb();

  public abstract void vibrate(RumbleType type, double value);

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
