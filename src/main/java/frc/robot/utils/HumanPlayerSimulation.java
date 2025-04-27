package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Random;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class HumanPlayerSimulation extends SubsystemBase {

  public enum HumanPlayerSimulationStrategy {
    NONE,
    ONLY_GROUND_NEAR,
    ONLY_GROUND_FAR,
    ONLY_SOURCE,
    GROUND_WHEN_OCCUPIED,
  }

  public enum HumanPlayerSource {
    RedLeft,
    RedRight,
    BlueLeft,
    BlueRight
  }

  private LoggedDashboardChooser<HumanPlayerSimulationStrategy> humanPlayerStrategyChooser;
  private List<HumanPlayer> humanPlayers = new ArrayList<>();

  public HumanPlayerSimulation() {
    if (Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL) {
      humanPlayerStrategyChooser = new LoggedDashboardChooser<>("HumanPlayerStrategy");
      EnumSet.allOf(HumanPlayerSimulationStrategy.class)
          .forEach(v -> humanPlayerStrategyChooser.addOption(v.name(), v));
      humanPlayerStrategyChooser.addDefaultOption(
          HumanPlayerSimulationStrategy.GROUND_WHEN_OCCUPIED.toString(),
          HumanPlayerSimulationStrategy.GROUND_WHEN_OCCUPIED);

      humanPlayers.add(
          new HumanPlayer(humanPlayerStrategyChooser.get(), HumanPlayerSource.RedLeft));
      humanPlayers.add(
          new HumanPlayer(humanPlayerStrategyChooser.get(), HumanPlayerSource.RedRight));
      humanPlayers.add(
          new HumanPlayer(humanPlayerStrategyChooser.get(), HumanPlayerSource.BlueLeft));
      humanPlayers.add(
          new HumanPlayer(humanPlayerStrategyChooser.get(), HumanPlayerSource.BlueRight));
    }
  }

  public static class HumanPlayer {

    private HumanPlayerSimulationStrategy strategy;
    private HumanPlayerSource source;

    public HumanPlayer(HumanPlayerSimulationStrategy strategy, HumanPlayerSource source) {
      this.strategy = strategy;
      this.source = source;
    }

    public void setStrategy(HumanPlayerSimulationStrategy strategy) {
      this.strategy = strategy;
    }

    public HumanPlayerSimulationStrategy getStrategy() {
      return strategy;
    }

    public void setSource(HumanPlayerSource source) {
      this.source = source;
    }

    public HumanPlayerSource getSource() {
      return source;
    }

    private double lastDropTime = 0;
    private int coralLeft = 30;
    private Random random = new Random();

    public void reset() {
      lastDropTime = 0;
      coralLeft = 30;
    }

    private Pose2d getSourcePose() {
      switch (source) {
        case RedLeft:
          return Constants.FieldConstants.SOURCE_RL;
        case RedRight:
          return Constants.FieldConstants.SOURCE_RR;
        case BlueLeft:
          return Constants.FieldConstants.SOURCE_BL;
        case BlueRight:
          return Constants.FieldConstants.SOURCE_BR;
        default:
          throw new IllegalArgumentException("Invalid source: " + source);
      }
    }

    private Pose3d getEjectPose(Pose2d target) {
      Pose3d sourcePose = new Pose3d(getSourcePose());

      Pose3d relativeTarget = new Pose3d(target).relativeTo(sourcePose);

      Pose3d coralPose =
          sourcePose.transformBy(
              new Transform3d(
                  MathUtil.clamp(relativeTarget.getX() + 0.1, -0.75, 0.75),
                  0,
                  1.16,
                  new Rotation3d(0, 0, 0)));

      return coralPose;
    }

    public void dropWithVelocity(Pose2d target, double velocity) {
      if (coralLeft <= 0) {
        return;
      }

      coralLeft--;

      var ejectPose = getEjectPose(target);
      var xyzVelocity = new Translation3d(0, velocity, 0).rotateBy(ejectPose.getRotation());

      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new GamePieceProjectile(
                      ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                      ejectPose.toPose2d().getTranslation(),
                      xyzVelocity.toTranslation2d(),
                      ejectPose.getZ(),
                      xyzVelocity.getZ(),
                      ejectPose.getRotation())
                  .withTouchGroundHeight(0.2)
                  .enableBecomesGamePieceOnFieldAfterTouchGround());
    }

    public static double GROUND_NEAR_VELOCITY = 1.0;
    public static double GROUND_FAR_VELOCITY = 4.0;
    public static double SOURCE_VELOCITY = 2.0;

    public void periodic() {
      switch (strategy) {
        case NONE:
          break;
        case ONLY_GROUND_NEAR:
          if (Timer.getTimestamp() - lastDropTime > random.nextDouble(1.0, 2.3)) {
            lastDropTime = Timer.getTimestamp();
            dropWithVelocity(RobotContainer.model.getRobot(), GROUND_NEAR_VELOCITY);
          }
          break;
        case ONLY_GROUND_FAR:
          if (Timer.getTimestamp() - lastDropTime > random.nextDouble(1.0, 2.3)) {
            lastDropTime = Timer.getTimestamp();
            dropWithVelocity(RobotContainer.model.getRobot(), GROUND_FAR_VELOCITY);
          }
          break;
        case ONLY_SOURCE:
          if (Timer.getTimestamp() - lastDropTime > random.nextDouble(1.4, 2.0)
              && RobotContainer.model
                      .getRobot()
                      .getTranslation()
                      .getDistance(getSourcePose().getTranslation())
                  < 1.0) {
            lastDropTime = Timer.getTimestamp();
            dropWithVelocity(RobotContainer.model.getRobot(), SOURCE_VELOCITY);
          }
          break;
        case GROUND_WHEN_OCCUPIED:
          if (Timer.getTimestamp() - lastDropTime > random.nextDouble(1.4, 2.0)) {
            if (RobotContainer.model
                    .getRobot()
                    .getTranslation()
                    .getDistance(getSourcePose().getTranslation())
                < 1.0) {
              lastDropTime = Timer.getTimestamp();
              dropWithVelocity(RobotContainer.model.getRobot(), SOURCE_VELOCITY);
            } else if (
            /*TODO: is occupied */ false) {
              lastDropTime = Timer.getTimestamp();
              dropWithVelocity(RobotContainer.model.getRobot(), GROUND_FAR_VELOCITY);
            }
          }
          break;
      }
    }
  }

  public void reset() {
    for (HumanPlayer player : humanPlayers) {
      player.reset();
    }
  }

  @Override
  public void simulationPeriodic() {
    for (HumanPlayer player : humanPlayers) {
      player.setStrategy(humanPlayerStrategyChooser.get());
      if (DriverStation.isEnabled()) {
        player.periodic();
      }
    }
  }
}
