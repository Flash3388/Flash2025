package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

public class RobotMap {

    private RobotMap() {}

    public static final int MIN_PRESSURE = 100;
    public static final int MAX_PRESSURE = 120;
    public static final int COMPRESSION_PORT = 1;

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
    public static final int CORAL_GRIPPER_MOTOR = 4;

    // coral arm
    public static final int ARM_CORAL_MOTOR = 5;
    public static final double ARM_CORAL_KP = 5.5;
    public static final double ARM_CORAL_KI = 0;
    public static final double ARM_CORAL_KD = 0;
    public static final double ARM_CORAL_IZONE = 0;
    public static final double ARM_CORAL_KF = 0.015;
    public static final double ARM_CORAL_TOLERANCE_POSITION_DEGREES = 1;
    public static final double ARM_CORAL_TOLERANCE_VELOCITY_RPM = 6.5;
    public static final double ARM_CORAL_ZERO_OFFSET = 0.6838749;
    public static final double ARM_CORAL_GEAR_RATIO = 200.0 / 1.0;
    public static final double ARM_CORAL_ANGLE_A = 40;
    public static final double ARM_CORAL_ANGLE_B = 230;
    public static final double ARM_CORAL_ANGLE_C = 50;
    public static final double ARM_CORAL_FF_POS_OFFSET = 62.3;

    // coral elevator
    public static final int CORAL_ELEVATOR_UPPER_LIMIT_SWITCH = 3;
    public static final int CORAL_ELEVATOR_LOWER_LIMIT_SWITCH = 2;
    public static final int CORAL_ELEVATOR_PISTON1_FORWARD_CHANNEL = 13;
    public static final int CORAL_ELEVATOR_PISTON1_REVERSE_CHANNEL = 15;
    // algae gripper
    public static final int ALGAE_GRIPPER_MOTOR1 = 7;
    public static final int ALGAE_GRIPPER_MOTOR2 = 6;

    public static final int ALGAE_GRIPPER_DIGITALINPUT = 0;

    // algae arm
    public static final int ALGEA_ARM_FORWARD_PISTON_FORWARD_CHANNEL = 12;
    public static final int ALGEA_ARM_FORWARD_PISTON_REVERSE_CHANNEL = 14;

    //vision system
    public static double LIMELIGHT_DISTANCE_TO_TARGET_LIMIT = 2.7;
    public static final double OFFSET_ON_STAND_REEF = 0.328676 / 2;
    public static final double OFFSET_ON_STAND_FEEDER = 0.6;
    public static final double OFFSET_REEF = 0.87 / 2 - 0.05;
    public static final double OFFSET_PROCESSOR = 0.87 / 2 - 0.05; // TODO: RECOMPUTE
    public static final double OFFSET_FEEDER = 0.71 / 2; // TODO: RECOMPUTE
    public static final PathConstraints CONSTRAINTS = new PathConstraints(
            2.6,1.5,Math.PI*2, Math.PI);

    public static final int[][] REEF_APRIL_TAGS_BY_ALLIANCE = {
            {9, 8, 7, 6, 11, 10}, // red alliance reef
            {22, 17, 18, 19, 20, 21}, // blue alliance reef
            {1,2}, // red feeders
            {13,12},// blue feeders
            {4,5}, // red cages
            {15,14} // blue cages
    };

    public static final int LEDS_ID = 0;
    public static final int LEDS_LENGTH = 5;
    public static final int LEDS_PER_METER = 60;
}
