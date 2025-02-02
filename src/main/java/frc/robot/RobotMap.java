package frc.robot;

public class RobotMap {

    private RobotMap() {}

    public static final int SWERVE_FRONT_LEFT_DRIVE = 11;
    public static final int SWERVE_FRONT_LEFT_STEER = 12;
    public static final int SWERVE_FRONT_LEFT_ENCODER = 10;
    public static final int SWERVE_BACK_LEFT_DRIVE = 31;
    public static final int SWERVE_BACK_LEFT_STEER = 32;
    public static final int SWERVE_BACK_LEFT_ENCODER = 30;
    public static final int SWERVE_FRONT_RIGHT_DRIVE = 21;
    public static final int SWERVE_FRONT_RIGHT_STEER = 22;
    public static final int SWERVE_FRONT_RIGHT_ENCODER = 20;
    public static final int SWERVE_BACK_RIGHT_DRIVE = 41;
    public static final int SWERVE_BACK_RIGHT_STEER = 42;
    public static final int SWERVE_BACK_RIGHT_ENCODER = 40;
    public static final int SWERVE_PIGEON = 2;

    // coral gripper
    public static final int CORAL_GRIPPER_LIMIT_SWITCH = 1;
    public static final int CORAL_GRIPPER_MOTOR = 1;

    // coral arm
    public static final int ARM_CORAL_MOTOR = 2;
    public static final double ARM_CORAL_KP = 0;
    public static final double ARM_CORAL_KI = 0;
    public static final double ARM_CORAL_KD = 0;
    public static final double ARM_CORAL_IZONE = 0;
    public static final double ARM_CORAL_KF = 0;
    public static final double ARM_CORAL_TOLERANCE_POSITION_DEGREES = 0.5;
    public static final double ARM_CORAL_TOLERANCE_VELOCITY_RPM = 5;
    public static final int ARM_CORAL_START_PULSE_US = 1;
    public static final int ARM_CORAL_END_PULSE_US = 1024;
    public static final double ARM_CORAL_ZERO_OFFSET = 0;
    public static final double ARM_CORAL_GEAR_RATIO = 0.005;
    public static final double ARM_CORAL_ANGLE_A = 0;
    public static final double ARM_CORAL_ANGLE_B = 0;

    // coral elevator
    public static final int CORAL_ELEVATOR_UPPER_LIMIT_SWITCH = 2;
    public static final int CORAL_ELEVATOR_LOWER_LIMIT_SWITCH = 3;
    public static final int CORAL_ELEVATOR_PISTON1_FORWARD_CHANNEL = 0;
    public static final int CORAL_ELEVATOR_PISTON1_REVERSE_CHANNEL = 1;
    public static final int CORAL_ELEVATOR_PISTON2_FORWARD_CHANNEL = 2;
    public static final int CORAL_ELEVATOR_PISTON2_REVERSE_CHANNEL = 3;

    // algae gripper
    public static final int ALGAE_GRIPPER_MOTOR = 4;
    public static final int ALGAE_GRIPPER_DIGITALINPUT = 4;

    // algae arm
    public static final int ALGAE_ARM_SWITCH_TOP = 5;
    public static final int ALGAE_ARM_SWITCH_BOTTOM = 6;
    public static final int ALGEA_ARM_FORWARD_PISTON_FORWARD_CHANNEL = 4;
    public static final int ALGEA_ARM_FORWARD_PISTON_REVERSE_CHANNEL = 5;
    public static final int ALGEA_ARM_BACKWARD_PISTON_FORWARD_CHANNEL = 6;
    public static final int ALGEA_ARM_BACKWARD_PISTON_REVERSE_CHANNEL = 7;
}
