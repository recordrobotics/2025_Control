package frc.robot.utils.camera;

public interface IVisionCamera {

  public String getName();

  public CameraType getCameraType();

  public boolean hasVision();

  public boolean isConnected();

  public double getConfidence();

  public double getUnprocessedConfidence();

  public int getNumTags();

  public VisionCameraEstimate getCurrentEstimate();

  public VisionCameraEstimate getUnsafeEstimate();

  public void setPipeline(int pipeline);

  public void updateEstimation(boolean trust);

  public void logValues(String id);
}
