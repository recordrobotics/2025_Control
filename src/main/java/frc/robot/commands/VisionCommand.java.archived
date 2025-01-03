package frc.robot.commands;

import java.util.HashSet;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class VisionCommand extends Command {
    public boolean hasVision;

    private final Command command;

    public VisionCommand(Command command, boolean hasVision) {
        this.hasVision = hasVision;
        this.command = command;
        if (hasVision) {
            m_requirements = command.getRequirements();
        } else {
            m_requirements = new HashSet<Subsystem>();
        }
    }

    public void setHasVision(boolean hasVision) {
        this.hasVision = hasVision;
        if (hasVision) {
            m_requirements = command.getRequirements();
        } else {
            m_requirements = new HashSet<Subsystem>();
        }
    }

    public boolean getHasVision() {
        return hasVision;
    }

    @Override
    public void initialize() {
        if (hasVision) {
            command.initialize();
        }
    }

    @Override
    public boolean isFinished() {
        if (hasVision) {
            return command.isFinished();
        } else {
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    protected Object clone() {
        return new VisionCommand(command, hasVision);
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof VisionCommand obj &&
                obj.command == command && obj.hasVision == hasVision;
    }

    @Override
    public int hashCode() {
        return command.hashCode() + Boolean.valueOf(hasVision).hashCode();
    }
}
