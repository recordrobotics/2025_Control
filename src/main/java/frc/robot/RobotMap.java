package frc.robot;

public final class RobotMap {
  public static final class Elevator {
    public static final int MOTOR_LEFT_ID = 12;
    public static final int MOTOR_RIGHT_ID = 13;
    public static final int BOTTOM_ENDSTOP_ID = 14; // TODO get actual DIO port number
    public static final int TOP_ENDSTOP_ID = 15; // TODO get actual DIO port number
  }

  public static final class CoralShooter {
    public static final int MOTOR_ID = 5; // TODO get actual CAN number
    public static final int LIMIT_SWITCH_ID = 6; // TODO get actual DIO port number
  }

  public static final class ElevatorAlgae {
    public static final int MOTOR_ID = 5; // TODO get actual CAN number
    public static final int LIMIT_SWITCH_ID = 6; // TODO get actual DIO port number
  }

  public static final class CoralIntake {
    public static final int SERVO_ID = 7;
    public static final int MOTOR_ID = 8; // TODO get actual CAN number
    public static final int LIMIT_SWITCH_ID = 9; // TODO get actual DIO port number
  }

  public static final class Lights {
    public static final int LED_ID = 0; // TODO get actual CAN number
  }

  public static final class GroundAlgae {
    public static final int MOTOR_ID = 10; // TODO get actual port number
    public static final int LIMIT_SWITCH_ID = 11; // TODO get actual DIO port number
  }
}
