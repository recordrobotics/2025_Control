package frc.robot.commands.manual;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.DoubleControl;
import frc.robot.subsystems.Crashbar;

public class ManualCrashbar extends Command {

    private Crashbar _crashbar;
    private DoubleControl _controls;

    public ManualCrashbar(Crashbar crashbar, DoubleControl controls) {
        _crashbar = crashbar;
        _controls = controls;
        addRequirements(crashbar);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("crashExtend", _controls.getCrashbarExtend());
        if (_controls.getCrashbarExtend()) {
            _crashbar.extend();
        } else if (_controls.getCrashbarRetract()) {
            _crashbar.retract();
        } else {
            _crashbar.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _crashbar.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
