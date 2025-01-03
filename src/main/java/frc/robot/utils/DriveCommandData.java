package frc.robot.utils;

/** An object that contains all relevant information for the drivetrain to drive */
public class DriveCommandData {

  public double xSpeed;
  public double ySpeed;
  public double rot;
  public boolean fieldRelative;

  // Constructor for an object that contains all relevant information for the drivetrain to drive */
  public DriveCommandData(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;
    this.fieldRelative = fieldRelative;
  }
}
