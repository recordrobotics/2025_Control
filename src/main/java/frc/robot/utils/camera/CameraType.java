package frc.robot.utils.camera;

public enum CameraType {
  Limelight2(1280, 960, 2, 74.36, 33, 0.007, 0.0005, 35, 5),
  Limelight3G(
      1280, 960, 1, 100, 38, 0.007, 0.0005, 35,
      5), // TODO: what is the actual fov of new lens we replaced
  SVPROGlobalShutter(1920, 1200, 1, 83, 33, 0.007, 0.0005, 35, 5);

  int width;
  int height;
  double downscale;
  double fov;
  double fps;
  double pxError;
  double pxErrorStdDev;
  double latencyMs;
  double latencyStdDevMs;

  CameraType(
      int width,
      int height,
      double downscale,
      double fov,
      double fps,
      double pxError,
      double pxErrorStdDev,
      double latencyMs,
      double latencyStdDevMs) {
    this.width = width;
    this.height = height;
    this.downscale = downscale;
    this.fov = fov;
    this.fps = fps;
    this.pxError = pxError;
    this.pxErrorStdDev = pxErrorStdDev;
    this.latencyMs = latencyMs;
    this.latencyStdDevMs = latencyStdDevMs;
  }

  public int getDetectorWidth() {
    return (int) Math.floor(width / downscale);
  }

  public int getDetectorHeight() {
    return (int) Math.floor(height / downscale);
  }
}
