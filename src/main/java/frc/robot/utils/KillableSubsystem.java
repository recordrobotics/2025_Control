package frc.robot.utils;

public abstract class KillableSubsystem extends ManagedSubsystemBase {
  public abstract void kill();

  public abstract void close() throws Exception;
}
