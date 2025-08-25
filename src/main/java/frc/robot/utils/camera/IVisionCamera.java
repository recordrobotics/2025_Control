package frc.robot.utils.camera;

public interface IVisionCamera {

    String getName();

    CameraType getCameraType();

    boolean hasVision();

    boolean isConnected();

    double getMeasurementStdDevs();

    int getNumTags();

    VisionCameraEstimate getCurrentEstimate();

    VisionCameraEstimate getUnsafeEstimate();

    void setPipeline(int pipeline);

    void updateEstimation(boolean trust, boolean ignore);

    void logValues(String id);
}
