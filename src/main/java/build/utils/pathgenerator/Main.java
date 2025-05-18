package build.utils.pathgenerator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.Game.CoralPosition;
import java.util.stream.Stream;

public final class Main {
  private Main() {}

  public static final Distance FIRST_STAGE_OFFSET = Meters.of(-0.6);

  public static void main(String... args) {
    for (var coral :
        Stream.of(CoralPosition.values()).filter(c -> c.name().startsWith("Blue")).toList()) {
      PathHelper.editLinkedWaypoint(
          "Reef" + coral.name().substring("Blue".length()),
          coral
              .getFirstStagePose()
              .transformBy(new Transform2d(FIRST_STAGE_OFFSET, Meters.of(0), Rotation2d.kZero)));
    }
  }
}
