package frc.robot.utils.assists;

public interface IAssist {

  /**
   * Apply the assist to the control
   *
   * @param control
   * @return true if the assist was applied, false otherwise
   */
  public boolean apply(DrivetrainControl control);

  public boolean isEnabled();

  public void setEnabled(boolean enabled);
}
