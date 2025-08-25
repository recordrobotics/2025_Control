package frc.robot.utils.camera;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.libraries.LimelightHelpers.PoseEstimate;
import frc.robot.utils.libraries.LimelightHelpers.RawFiducial;
import java.util.Arrays;

public record VisionCameraEstimate(
        Pose2d pose,
        double timestampSeconds,
        double latency,
        int tagCount,
        double avgTagDist,
        double avgTagArea,
        RawVisionFiducial[] rawFiducials,
        boolean isConstrained) {

    public record RawVisionFiducial(int id, double area, double distToCamera, double distToRobot, double ambiguity) {
        public RawVisionFiducial(RawFiducial limelightFiducial) {
            this(
                    limelightFiducial.id,
                    limelightFiducial.ta,
                    limelightFiducial.distToCamera,
                    limelightFiducial.distToRobot,
                    limelightFiducial.ambiguity);
        }
    }

    public VisionCameraEstimate() {
        this(new Pose2d(), 0, 0, 0, 0, 0, new RawVisionFiducial[0], false);
    }

    private static RawVisionFiducial[] convertRawFiducials(RawFiducial[] rawFiducials) {
        RawVisionFiducial[] converted = new RawVisionFiducial[rawFiducials.length];
        for (int i = 0; i < rawFiducials.length; i++) {
            converted[i] = new RawVisionFiducial(rawFiducials[i]);
        }
        return converted;
    }

    public VisionCameraEstimate(PoseEstimate limelightEstimate) {
        this(
                limelightEstimate.pose,
                limelightEstimate.timestampSeconds,
                limelightEstimate.latency,
                limelightEstimate.tagCount,
                limelightEstimate.avgTagDist,
                limelightEstimate.avgTagArea,
                convertRawFiducials(limelightEstimate.rawFiducials),
                limelightEstimate.isMegaTag2);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        VisionCameraEstimate that = (VisionCameraEstimate) obj;
        return Double.compare(that.timestampSeconds, timestampSeconds) == 0
                && Double.compare(that.latency, latency) == 0
                && tagCount == that.tagCount
                && Double.compare(that.avgTagDist, avgTagDist) == 0
                && Double.compare(that.avgTagArea, avgTagArea) == 0
                && isConstrained == that.isConstrained
                && pose.equals(that.pose)
                && Arrays.equals(rawFiducials, that.rawFiducials);
    }

    @Override
    public int hashCode() {
        int result = pose.hashCode();
        result = 31 * result + Double.hashCode(timestampSeconds);
        result = 31 * result + Double.hashCode(latency);
        result = 31 * result + tagCount;
        result = 31 * result + Double.hashCode(avgTagDist);
        result = 31 * result + Double.hashCode(avgTagArea);
        result = 31 * result + Arrays.hashCode(rawFiducials);
        result = 31 * result + Boolean.hashCode(isConstrained);
        return result;
    }

    @Override
    public String toString() {
        return "VisionCameraEstimate{" + "pose="
                + pose + ", timestampSeconds="
                + timestampSeconds + ", latency="
                + latency + ", tagCount="
                + tagCount + ", avgTagDist="
                + avgTagDist + ", avgTagArea="
                + avgTagArea + ", rawFiducials="
                + Arrays.toString(rawFiducials) + ", isConstrained="
                + isConstrained + '}';
    }
}
