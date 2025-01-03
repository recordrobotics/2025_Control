package frc.robot;

public final class RobotMap {

    public static final class Aquisition {
        public static final int lower = 0;
        public static final int upper = 0;
    }

    public static final class Climbers {
        public static final int left = 0;
        public static final int right = 0;
    }

    public static final class Shooter {
        public static final int flywheel = 0;
    }

    public static final class swerve {
        // todo change to correct ports
        public static final int[] SPEED_MOTOR_DEVICE_IDS = { 4, 2, 6, 8 };
        public static final int[] DIRECTION_MOTOR_DEVICE_IDS = { 3, 1, 5, 7 };
        public static final int[] ENCODER_DEVICE_IDS = { 4,3,2,1 };
    }

    public static class Control {
        public static final int SINGLE_GAMEPAD = 0;
    }
}
