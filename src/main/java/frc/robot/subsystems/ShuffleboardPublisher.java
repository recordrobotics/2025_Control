package frc.robot.subsystems;

public interface ShuffleboardPublisher {
  public void setupShuffleboard();

  public static void setup(ShuffleboardPublisher... publishers) {
    for (ShuffleboardPublisher publisher : publishers) {
      publisher.setupShuffleboard();
    }
  }
}
