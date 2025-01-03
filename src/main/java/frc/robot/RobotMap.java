package frc.robot;

public final class RobotMap {

    public static final class Aquisition {
        public static final int LOWER_ACQUISITION_MOTOR_ID = -1;
        public static final int UPPER_ACQUISITION_MOTOR_ID = -1;
    }

    public static final class Climbers {
        public static final int LEFT_FORWARD_PORT = 1;
        public static final int LEFT_REVERSE_PORT = 0;
    }

    public static final class Crashbar {
        public static final int FORWARD_PORT = 3;
        public static final int REVERSE_PORT = 2;
    }

    public static final class Shooter {
        public static final int FLYWHEEL_MOTOR_LEFT_DEVICE_ID = 9;
        public static final int FLYWHEEL_MOTOR_RIGHT_DEVICE_ID = 10;
    }

    /**
     * Maps the motors and encoders of each module to their correct IDs. Currently not used. 
     */
    public static final class Swerve {
        public static final int DRIVE_MOTOR_FRONT_LEFT_DEVICE_ID = 2;
        public static final int DRIVE_MOTOR_FRONT_RIGHT_DEVICE_ID = 4;
        public static final int DRIVE_MOTOR_BACK_LEFT_DEVICE_ID = 8;
        public static final int DRIVE_MOTOR_BACK_RIGHT_DEVICE_ID = 6;

        public static final int TURN_MOTOR_FRONT_LEFT_DEVICE_ID = 1;
        public static final int TURN_MOTOR_FRONT_RIGHT_DEVICE_ID = 3;
        public static final int TURN_MOTOR_BACK_LEFT_DEVICE_ID = 7;
        public static final int TURN_MOTOR_BACK_RIGHT_DEVICE_ID = 5;

        public static final int ABS_ENCODER_FRONT_LEFT_DEVICE_ID = 2;
        public static final int ABS_ENCODER_FRONT_RIGHT_DEVICE_ID = 3;
        public static final int ABS_ENCODER_BACK_LEFT_DEVICE_ID = 1;
        public static final int ABS_ENCODER_BACK_RIGHT_DEVICE_ID = 4;
    }

    public static class Control {
        public static final int STICKPAD_PORT = 0;
        public static final int XBOX_PORT = 1;
    }
}
