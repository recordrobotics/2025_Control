package frc.robot.utils.maplesim.multiplayer.messages;

import com.jfastnet.messages.GenericMessage;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.utils.maplesim.multiplayer.Constants;
import org.littletonrobotics.junction.Logger;

public class RobotStateUpdateMessage extends GenericMessage {

    private double[] pose;
    private double[] mechanismPoses;

    public RobotStateUpdateMessage() {
        this.pose = new double[] {0.0, 0.0, 0.0};
        this.mechanismPoses = new double[] {};
    }

    public RobotStateUpdateMessage(double[] pose, double[] mechanismPoses) {
        this.pose = pose;
        this.mechanismPoses = mechanismPoses;
    }

    public RobotStateUpdateMessage(Pose2d pose, Pose3d[] mechanismPoses) {
        this.pose = new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        this.mechanismPoses = new double[mechanismPoses.length * 7]; /* x,y,z,rx,ry,rz,rw */
        for (int i = 0; i < mechanismPoses.length; i++) {
            this.mechanismPoses[i * 7] = mechanismPoses[i].getX();
            this.mechanismPoses[i * 7 + 1] = mechanismPoses[i].getY();
            this.mechanismPoses[i * 7 + 2] = mechanismPoses[i].getZ();
            this.mechanismPoses[i * 7 + 3] =
                    mechanismPoses[i].getRotation().getQuaternion().getW();
            this.mechanismPoses[i * 7 + 4] =
                    mechanismPoses[i].getRotation().getQuaternion().getX();
            this.mechanismPoses[i * 7 + 5] =
                    mechanismPoses[i].getRotation().getQuaternion().getY();
            this.mechanismPoses[i * 7 + 6] =
                    mechanismPoses[i].getRotation().getQuaternion().getZ();
        }
    }

    public Pose2d getPose() {
        return new Pose2d(pose[0], pose[1], Rotation2d.fromRadians(pose[2]));
    }

    public Pose3d[] getMechanismPoses() {
        Pose3d[] poses = new Pose3d[mechanismPoses.length / 7];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = new Pose3d(
                    mechanismPoses[i * 7],
                    mechanismPoses[i * 7 + 1],
                    mechanismPoses[i * 7 + 2],
                    new Rotation3d(new Quaternion(
                            mechanismPoses[i * 7 + 3],
                            mechanismPoses[i * 7 + 4],
                            mechanismPoses[i * 7 + 5],
                            mechanismPoses[i * 7 + 6])));
        }
        return poses;
    }

    @Override
    public void process(Object context) {
        if (!Constants.isInServerMode()) {
            String root = "MapleSim/Multiplayer/" + getSenderId() + "/";
            Logger.recordOutput(root + "Pose", getPose());
            Logger.recordOutput(root + "MechanismPoses", getMechanismPoses());
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
