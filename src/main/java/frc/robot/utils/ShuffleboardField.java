package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleboardField {
    private static final Field2d field = new Field2d();

    private static void putData() {
        SmartDashboard.putData(field);
    }

    public static void setRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
        SmartDashboard.putNumber("x", pose.getX());
        SmartDashboard.putNumber("y", pose.getY());
        putData();
    }

    public static void setTabletPos(double x, double y) {
        field.getObject("Tablet Target").setPose(new Pose2d(x, y, new Rotation2d(0)));
        putData();
    }

    public static void setVisionPoses(Pose2d[] poses) {
        field.getObject("Vision Pose").setPoses(poses);
    }

    public static void setVisionPose(Pose2d pose) {
        field.getObject("Vision Pose").setPose(pose);
    }
}
