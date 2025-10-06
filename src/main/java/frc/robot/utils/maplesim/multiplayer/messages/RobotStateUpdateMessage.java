package frc.robot.utils.maplesim.multiplayer.messages;

import com.jfastnet.messages.GenericMessage;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.maplesim.multiplayer.Constants;
import org.littletonrobotics.junction.Logger;

public class RobotStateUpdateMessage extends GenericMessage {

    private double[] pose;

    public RobotStateUpdateMessage() {
        this.pose = new double[] {0.0, 0.0, 0.0};
    }

    public RobotStateUpdateMessage(double[] pose) {
        this.pose = pose;
    }

    public RobotStateUpdateMessage(Pose2d pose) {
        this.pose = new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

    public Pose2d getPose() {
        return new Pose2d(pose[0], pose[1], Rotation2d.fromRadians(pose[2]));
    }

    @Override
    public void process(Object context) {
        if (!Constants.isInServerMode()) {
            Logger.recordOutput("MapleSim/Multiplayer/" + getSenderId() + "/Pose", getPose());
        }
    }

    @Override
    public ReliableMode getReliableMode() {
        return ReliableMode.UNRELIABLE;
    }

    @Override
    public boolean broadcast() {
        return true;
    }

    @Override
    public boolean sendBroadcastBackToSender() {
        return false;
    }

    @Override
    public int getTimeOut() {
        return 1000;
    }

    static final long serialVersionUID = 1L;
}
