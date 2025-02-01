package frc.robot.dashboard;

import java.util.Map;

/**
 * Used for tuning using shuffleboard (contains the current value and target of whatever you are
 * trying to tune) For example can be used for motor PID tuning, velocity tuning, feed forward
 * tuning, etc.
 */
public class TuningData {
  public Double current;
  public Double target;

  public TuningData(double current, double target) {
    this.current = current;
    this.target = target;
  }

  public double[] toArray() {
    return new double[] {current, target};
  }

  /** Map TuningData entries to a single big double[] used by shuffleboard */
  public static <T> double[] MapToArray(Map<T, TuningData> map) {
    var values = map.values().toArray();
    double[] db = new double[values.length * 2];
    for (int i = 0; i < values.length; i++) {
      if (values[i] instanceof TuningData p /* safe cast from Object */) {
        db[i * 2] = p.current;
        db[i * 2 + 1] = p.target;
      }
    }
    return db;
  }
}
