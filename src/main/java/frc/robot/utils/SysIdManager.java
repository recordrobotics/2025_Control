package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import java.util.function.Function;
import java.util.function.Supplier;

public class SysIdManager {

    public static final double IN_BETWEEN_STEP_DELAY = 0.4;

    public static Command createCommand(
            Function<Direction, Command> quasistatic, Function<Direction, Command> dynamic) {
        return new InstantCommand()
                .andThen(quasistatic.apply(Direction.kForward).andThen(new WaitCommand(IN_BETWEEN_STEP_DELAY)))
                .andThen(quasistatic.apply(Direction.kReverse).andThen(new WaitCommand(IN_BETWEEN_STEP_DELAY)))
                .andThen(dynamic.apply(Direction.kForward).andThen(new WaitCommand(IN_BETWEEN_STEP_DELAY)))
                .andThen(dynamic.apply(Direction.kReverse).andThen(new WaitCommand(IN_BETWEEN_STEP_DELAY)));
    }

    public static SysIdRoutine getSysIdRoutine() {
        return SysIdRoutine.NONE;
    }

    public enum SysIdRoutine {
        NONE,
        CLIMBER(() -> RobotContainer.climber::sysIdQuasistatic, () -> RobotContainer.climber::sysIdDynamic),
        CORAL_INTAKE_ARM(
                () -> RobotContainer.coralIntake::sysIdQuasistaticArm,
                () -> RobotContainer.coralIntake::sysIdDynamicArm),
        CORAL_INTAKE_WHEEL(
                () -> RobotContainer.coralIntake::sysIdQuasistaticWheel,
                () -> RobotContainer.coralIntake::sysIdDynamicWheel),
        DRIVETRAIN_TURN(
                () -> RobotContainer.drivetrain::sysIdQuasistaticTurnMotors,
                () -> RobotContainer.drivetrain::sysIdDynamicTurnMotors),
        DRIVETRAIN_SPIN(
                () -> RobotContainer.drivetrain::sysIdQuasistaticDriveMotorsSpin,
                () -> RobotContainer.drivetrain::sysIdDynamicDriveMotorsSpin),
        DRIVETRAIN_FORWARD(
                () -> RobotContainer.drivetrain::sysIdQuasistaticDriveMotorsForward,
                () -> RobotContainer.drivetrain::sysIdDynamicDriveMotorsForward),
        ELEVATOR(() -> RobotContainer.elevator::sysIdQuasistatic, () -> RobotContainer.elevator::sysIdDynamic),
        ELEVATOR_ARM(() -> RobotContainer.elevator::sysIdQuasistatic, () -> RobotContainer.elevator::sysIdDynamic),
        ELEVATOR_HEAD(
                () -> RobotContainer.elevatorHead::sysIdQuasistatic, () -> RobotContainer.elevatorHead::sysIdDynamic);

        private final boolean enabled;
        private final Supplier<Function<Direction, Command>> quasistatic;
        private final Supplier<Function<Direction, Command>> dynamic;

        SysIdRoutine(
                Supplier<Function<Direction, Command>> quasistatic, Supplier<Function<Direction, Command>> dynamic) {
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
                return SysIdManager.createCommand(quasistatic.get(), dynamic.get());
            } else {
                return Commands.none();
            }
        }
    }
}
