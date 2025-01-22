package frc.robot;

import com.revrobotics.spark.SparkMax;

public class RobotMap {

    private RobotMap() {}

    // coral gripper
    public static final int CORAL_GRIPPER_LIMIT_SWITCH = 1;
    public static final int CORAL_GRIPPER_MOTOR = 2;

    // coral arm
    public static final int ARM_CORAL_MOTOR = 1;
    public static final double ARM_CORAL_KP = 0;
    public static final double ARM_CORAL_KI = 0;
    public static final double ARM_CORAL_KD = 0;
    public static final double ARM_CORAL_IZONE = 0;
    public static final double ARM_CORAL_TOLERANCE_POSITION_DEGREES = 0.5;
    public static final double ARM_CORAL_TOLERANCE_VELOCITY_RPM = 5;
    public static final int ARM_CORAL_START_PULSE_US = 1;
    public static final int ARM_CORAL_END_PULSE_US = 1024;
    public static final double ARM_CORAL_ZERO_OFFSET = 0;
    public static final double ARM_CORAL_KF = 0;

    // coral elevator
    public static final int CORAL_ELEVATOR_UPPER_LIMIT_SWITCH = 1;
    public static final int CORAL_ELEVATOR_LOWER_LIMIT_SWITCH = 2;
    public static final int CORAL_ELEVATOR_PISTON_FORWARD_CHANNEL = 1;
    public static final int CORAL_ELEVATOR_PISTON_REVERSE_CHANNEL = 1;

    // algae gripper
    public static final int ALGAE_GRIPPER_MOTOR = 0;
    public static final int ALGAE_GRIPPER_DIGITALINPUT = 0;

    // algae arm
    public static final int ALGAE_ARM_SWITCH_TOP = 0;
    public static final int ALGAE_ARM_SWITCH_BOTTOM = 0;
    public static final int ALGEA_ARM_FORWARD_PISTON_FORWARD_CHANNEL = 0;
    public static final int ALGEA_ARM_FORWARD_PISTON_REVERSE_CHANI = 0;
    public static final int ALGEA_ARM_BACKWARD_PISTON_FORWARD_CHANNEL = 0;
    public static final int ALGEA_ARM_BACKWARD_PISTON_REVERSE_CHANI = 0;
}
