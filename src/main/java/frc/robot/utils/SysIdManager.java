package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import java.util.function.Function;

public class SysIdManager {

  public static Command createCommand(
      Function<Direction, Command> quasistatic, Function<Direction, Command> dynamic) {
    return new InstantCommand()
        .andThen(quasistatic.apply(Direction.kForward).andThen(new WaitCommand(0.4)))
        .andThen(quasistatic.apply(Direction.kReverse).andThen(new WaitCommand(0.4)))
        .andThen(dynamic.apply(Direction.kForward).andThen(new WaitCommand(0.4)))
        .andThen(dynamic.apply(Direction.kReverse).andThen(new WaitCommand(0.4)));
  }

  public static SysIdRoutine getSysIdRoutine() {
    return SysIdRoutine.None;
  }

  public static enum SysIdRoutine {
    None,
    Climber(RobotContainer.climber::sysIdQuasistatic, RobotContainer.climber::sysIdDynamic),
    CoralIntakeArm(
        RobotContainer.coralIntake::sysIdQuasistaticArm,
        RobotContainer.coralIntake::sysIdDynamicArm),
    CoralIntakeWheel(
        RobotContainer.coralIntake::sysIdQuasistaticWheel,
        RobotContainer.coralIntake::sysIdDynamicWheel),
    DrivetrainTurn(
        RobotContainer.drivetrain::sysIdQuasistaticTurnMotors,
        RobotContainer.drivetrain::sysIdDynamicTurnMotors),
    DrivetrainSpin(
        RobotContainer.drivetrain::sysIdQuasistaticDriveMotorsSpin,
        RobotContainer.drivetrain::sysIdDynamicDriveMotorsSpin),
    DrivetrainForward(
        RobotContainer.drivetrain::sysIdQuasistaticDriveMotorsForward,
        RobotContainer.drivetrain::sysIdDynamicDriveMotorsForward),
    Elevator(RobotContainer.elevator::sysIdQuasistatic, RobotContainer.elevator::sysIdDynamic),
    ElevatorArm(RobotContainer.elevator::sysIdQuasistatic, RobotContainer.elevator::sysIdDynamic),
    ElevatorHead(
        RobotContainer.elevatorHead::sysIdQuasistatic, RobotContainer.elevatorHead::sysIdDynamic);

    private final boolean enabled;
    private final Function<Direction, Command> quasistatic;
    private final Function<Direction, Command> dynamic;

    SysIdRoutine(Function<Direction, Command> quasistatic, Function<Direction, Command> dynamic) {
      this.quasistatic = quasistatic;
      this.dynamic = dynamic;
      enabled = true;
    }

    SysIdRoutine() {
      this.quasistatic = null;
      this.dynamic = null;
      enabled = false;
    }

    public boolean isEnabled() {
      return enabled;
    }

    public Command createCommand() {
      if (enabled) {
        return SysIdManager.createCommand(quasistatic, dynamic);
      } else {
        return Commands.none();
      }
    }
  }
}
