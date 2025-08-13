package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoPath;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.recordrobotics.ruckig.InputParameter3;
import org.recordrobotics.ruckig.OutputParameter3;
import org.recordrobotics.ruckig.Ruckig3;
import org.recordrobotics.ruckig.Trajectory3.KinematicState;
import org.recordrobotics.ruckig.enums.DurationDiscretization;
import org.recordrobotics.ruckig.enums.Result;
import org.recordrobotics.ruckig.enums.Synchronization;

public class RuckigAlign extends Command {

    private static final Ruckig3 ruckig = new Ruckig3();
    private static final InputParameter3 input = new InputParameter3();
    private static final OutputParameter3 output = new OutputParameter3();

    static {
        input.setDurationDiscretization(DurationDiscretization.Discrete);
        input.setDefaultSynchronization(Synchronization.Phase);
        input.setPerDoFSynchronization(
                new Synchronization[] {Synchronization.Phase, Synchronization.Phase, Synchronization.None});
    }

    private final Supplier<KinematicState> targetStateSupplier;
    private final double[] maxVelocity;
    private final double[] maxAcceleration;
    private final double[] maxJerk;

    private Result result;

    private final PIDController xpid = new PIDController(6, 0, 0.02);
    private final PIDController ypid = new PIDController(6, 0, 0.02);
    private final PIDController rpid = new PIDController(2.48, 0, 0.01);

    public RuckigAlign(
            Supplier<KinematicState> targetStateSupplier,
            double[] maxVelocity,
            double[] maxAcceleration,
            double[] maxJerk) {
        this.targetStateSupplier = targetStateSupplier;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;

        xpid.setTolerance(0.01, 0.05);
        ypid.setTolerance(0.01, 0.05);
        rpid.setTolerance(Math.toRadians(1), Math.toRadians(5));

        addRequirements(RobotContainer.drivetrain);
    }

    private static double[] pose2dToArray(Pose2d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

    private static double[] chassisSpeedsToArray(ChassisSpeeds speeds) {
        return new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond};
    }

    private static void setTargetState(KinematicState state) {
        input.setTargetPosition(state.position());
        input.setTargetVelocity(state.velocity());
        input.setTargetAcceleration(state.acceleration());
    }

    private void reset() {
        input.setCurrentPosition(pose2dToArray(RobotContainer.poseSensorFusion.getEstimatedPosition()));
        input.setCurrentVelocity(chassisSpeedsToArray(RobotContainer.drivetrain.getChassisSpeeds()));
        input.setCurrentAcceleration(chassisSpeedsToArray(RobotContainer.drivetrain.getChassisAcceleration()));
        xpid.reset();
        ypid.reset();
        rpid.reset();
        result = Result.Working;
    }

    @Override
    public void initialize() {
        input.setMaxVelocity(maxVelocity);
        input.setMaxAcceleration(maxAcceleration);
        input.setMaxJerk(maxJerk);

        reset();
        setTargetState(targetStateSupplier.get());
    }

    @Override
    public void execute() {
        setTargetState(targetStateSupplier.get());

        result = ruckig.update(input, output);

        Pose2d currentPose = RobotContainer.poseSensorFusion.getEstimatedPosition();

        double[] newPosition = output.getNewPosition();
        double[] newVelocity = output.getNewVelocity();

        Logger.recordOutput(
                "Ruckig/Setpoint", new Pose2d(newPosition[0], newPosition[1], new Rotation2d(newPosition[2])));

        // Calculate the new velocities using PID and velocity feedforward
        double vx = xpid.calculate(currentPose.getX(), newPosition[0]) + newVelocity[0];
        double vy = ypid.calculate(currentPose.getY(), newPosition[1]) + newVelocity[1];
        double vr = rpid.calculate(currentPose.getRotation().getRadians(), newPosition[2]) + newVelocity[2];

        AutoPath.controlModifier.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(vx, vy, vr), currentPose.getRotation()));

        output.passToInput(input);

        double ex = currentPose.getX() - newPosition[0];
        double ey = currentPose.getY() - newPosition[1];
        double er = currentPose.getRotation().getRadians() - newPosition[2];

        Logger.recordOutput("Ruckig/Errors", new double[] {ex, ey, er});

        // If the error is too large, reset the trajectory to current position for more accurate motion
        if (ex * ex + ey * ey > 1.0 /* TODO: replace with smaller value */ || Math.abs(er) > Math.PI) {
            reset();
        }
    }

    @Override
    public boolean isFinished() {
        return result != Result.Working && xpid.atSetpoint() && ypid.atSetpoint() && rpid.atSetpoint();
    }
}
