package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CoralElevator extends SubsystemBase {

    private static final double MAX_UPPER_INPUT = 4100;
    private static final double MIN_UPPER_INPUT = 3000;
    private static final double MIN_LOWER_INPUT = 3000;
    private static final double MAX_LOWER_INPUT = 4100;

    private final DoubleSolenoid piston1;
    private final AnalogInput UpperLimitSwitch;
    private final AnalogInput LowerLimitSwitch;

    public CoralElevator() {
        piston1 = new DoubleSolenoid(1,PneumaticsModuleType.REVPH,RobotMap.CORAL_ELEVATOR_PISTON1_FORWARD_CHANNEL,RobotMap.CORAL_ELEVATOR_PISTON1_REVERSE_CHANNEL);
        UpperLimitSwitch = new AnalogInput(RobotMap.CORAL_ELEVATOR_UPPER_LIMIT_SWITCH);
        LowerLimitSwitch = new AnalogInput(RobotMap.CORAL_ELEVATOR_LOWER_LIMIT_SWITCH);
    }

    public boolean isRaised() {
        return isConstrained(UpperLimitSwitch.getValue(),MIN_UPPER_INPUT,MAX_UPPER_INPUT);
    }

    public boolean isLowered() {
        return isConstrained(LowerLimitSwitch.getValue(),MIN_LOWER_INPUT,MAX_LOWER_INPUT);
    }

    public void raise() {
        piston1.set(DoubleSolenoid.Value.kForward);
    }

    public void lower() {
        piston1.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop() {
        piston1.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    public void periodic() {
    }

    private boolean isConstrained(double value, double min, double max){
        return value <= max && value >= min;
    }
}
