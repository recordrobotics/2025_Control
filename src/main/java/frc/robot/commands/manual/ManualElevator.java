package frc.robot.commands.manual;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;

public class ManualElevator extends Command {

    private static final double DEADBAND = 0.001;

    private double height = 0;
    private double angle = 0;

    public ManualElevator() {
        addRequirements(RobotContainer.elevator);
    }

    @Override
    public void initialize() {
        height = RobotContainer.elevator.getCurrentHeight();
        angle = RobotContainer.elevator.getArmAngle();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        AbstractControl controls = DashboardUI.Overview.getControl();

        LinearVelocity manualElevatorVelocity = controls.getManualElevatorVelocity();
        double delta = manualElevatorVelocity
                .times(Seconds.of(RobotContainer.ROBOT_PERIODIC))
                .in(Meters);
        height += delta;
        height = MathUtil.clamp(height, Constants.Elevator.STARTING_HEIGHT, Constants.Elevator.MAX_HEIGHT);

        AngularVelocity manualElevatorArmVelocity = controls.getManualElevatorArmVelocity();
        double armDelta = manualElevatorArmVelocity
                .times(Seconds.of(RobotContainer.ROBOT_PERIODIC))
                .in(Radians);
        angle += armDelta;
        angle = MathUtil.clamp(angle, Constants.ElevatorArm.MIN_POS, Constants.ElevatorArm.MAX_POS);

        if (Math.abs(delta) > DEADBAND || Math.abs(armDelta) > DEADBAND) {
            RobotContainer.elevator.set(height, angle);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
