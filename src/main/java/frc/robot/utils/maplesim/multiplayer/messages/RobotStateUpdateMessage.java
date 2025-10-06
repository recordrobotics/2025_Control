package frc.robot.utils.maplesim.multiplayer.messages;

import com.jfastnet.messages.Message;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.utils.maplesim.multiplayer.messages.context.MapleSimContext;
import org.littletonrobotics.junction.Logger;

public class RobotStateUpdateMessage extends Message<MapleSimContext> {

    private double[] pose;
    private double[] mechanismPoses;
    private double[] robotCoral;

    public RobotStateUpdateMessage() {
        this.pose = new double[] {0.0, 0.0, 0.0};
        this.mechanismPoses = new double[] {};
        this.robotCoral = new double[] {};
    }

    public RobotStateUpdateMessage(double[] pose, double[] mechanismPoses, double[] robotCoral) {
        this.pose = pose;
        this.mechanismPoses = mechanismPoses;
        this.robotCoral = robotCoral;
    }

    public RobotStateUpdateMessage(Pose2d pose, Pose3d[] mechanismPoses, Pose3d robotCoral) {
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

        if (robotCoral == null) {
            this.robotCoral = new double[] {};
        } else {
            this.robotCoral = new double[7];
            this.robotCoral[0] = robotCoral.getX();
            this.robotCoral[1] = robotCoral.getY();
            this.robotCoral[2] = robotCoral.getZ();
            this.robotCoral[3] = robotCoral.getRotation().getQuaternion().getW();
            this.robotCoral[4] = robotCoral.getRotation().getQuaternion().getX();
            this.robotCoral[5] = robotCoral.getRotation().getQuaternion().getY();
            this.robotCoral[6] = robotCoral.getRotation().getQuaternion().getZ();
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

    public Pose3d getRobotCoral() {
        if (robotCoral.length != 7) {
            return null;
        }
        return new Pose3d(
                robotCoral[0],
                robotCoral[1],
                robotCoral[2],
                new Rotation3d(new Quaternion(robotCoral[3], robotCoral[4], robotCoral[5], robotCoral[6])));
    }

    @Override
    public void process(MapleSimContext context) {
        if (!context.isServer()) {
            String root = "MapleSim/Multiplayer/" + getSenderId() + "/";
            Logger.recordOutput(root + "Pose", getPose());
            Logger.recordOutput(root + "MechanismPoses", getMechanismPoses());
            context.addRobotCoral(getReceiverId(), getRobotCoral());
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
