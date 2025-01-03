package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardField;

import java.util.Arrays;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {

    private static NetworkTable _networkTable;

    private static Pose2d lastDetectedPose;
    private static int lastDetectedTagID;

    private static boolean hasTag = false;

    public Vision() {
        _networkTable = NetworkTableInstance.getDefault().getTable("JetsonVision");
    }

    @Override
    public void periodic() {

        try {
            hasTag = _networkTable.getValue("Has pose").getBoolean();
        } catch (Exception e) {
            hasTag = false;
        }

        if (hasTag) {

            double[] lastPoseArray = _networkTable.getValue("Pose").getDoubleArray();

            double lastX = lastPoseArray[0];
            double lastY = lastPoseArray[1];
            double lastRot = lastPoseArray[2];

            // Gets last detected pose
            lastDetectedPose = new Pose2d(
                    new Translation2d(lastX, lastY),
                    new Rotation2d(lastRot));

            // Gets the ID of the last detected tag
            lastDetectedTagID = (int) _networkTable.getValue("Tag ID").getDouble();
        }

        SmartDashboard.putBoolean("HAS POSE", hasTag);
        SmartDashboard.putNumber("Last Id", lastDetectedTagID);

        if (lastDetectedPose != null) {
            // var poseKeys = _networkTable.getKeys().stream().filter((v) ->
            // v.startsWith("Pose")).toList();
            // Pose2d[] poses = new Pose2d[poseKeys.size()];
            // for (int i = 0; i < poseKeys.size(); i++) {
            // var dt = _networkTable.getValue(poseKeys.get(i)).getDoubleArray();
            // poses[i] = new Pose2d(new Translation2d(dt[0], dt[1]),
            // new Rotation2d(dt[2]));
            // }
            ShuffleboardField.setVisionPose(lastDetectedPose);
        }
    }

    public boolean checkForSpecificTags(Integer[] allowedTags) {
        if (hasTag) {
            return Arrays.asList(allowedTags).contains(lastDetectedTagID);
        }
        return false;
    }

    public Pose2d getLastPose() {
        return lastDetectedPose;
    }

}