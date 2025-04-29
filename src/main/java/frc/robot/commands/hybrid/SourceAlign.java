package frc.robot.commands.hybrid;

import edu.wpi.first.wpilibj2.command.Command;

public class SourceAlign {
  public static Command create(boolean useTranslation) {
    return new AlignToPose(null, useTranslation); // TODO
  }
}
