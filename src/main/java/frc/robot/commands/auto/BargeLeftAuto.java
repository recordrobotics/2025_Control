package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Align;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class BargeLeftAuto extends SequentialCommandGroup {

  public BargeLeftAuto() throws FileVersionException, IOException, ParseException {
    addCommands(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("BargeLeftToL4")),
        Align.create(0.01, 0.05, true).withTimeout(1.0));
  }
}
