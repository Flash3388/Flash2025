package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CoralElevator extends SubsystemBase {

    public final DoubleSolenoid piston1;
    //private final DoubleSolenoid piston2;
    private final AnalogInput UpperLimitSwitch;
    private final double MAX_UPPER_INPUT = 1400;
    private final double MIN_UPPER_INPUT = 100;
    private final double MIN_LOWER_INPUT = 100;
    private final double MAX_LOWER_INPUT = 1400;
    private final AnalogInput LowerLimitSwitch;

    public CoralElevator() {
        piston1 = new DoubleSolenoid(1,PneumaticsModuleType.REVPH,RobotMap.CORAL_ELEVATOR_PISTON1_FORWARD_CHANNEL,RobotMap.CORAL_ELEVATOR_PISTON1_REVERSE_CHANNEL);
        //piston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.CORAL_ELEVATOR_PISTON2_FORWARD_CHANNEL, RobotMap.CORAL_ELEVATOR_PISTON2_REVERSE_CHANNEL);
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
        //piston2.set(DoubleSolenoid.Value.kForward);
    }

    public void lower() {
        piston1.set(DoubleSolenoid.Value.kReverse);
       // piston2.set(DoubleSolenoid.Value.kReverse);
    }
    public void stop() {
        piston1.set(DoubleSolenoid.Value.kOff);
       // piston2.set(DoubleSolenoid.Value.kOff);
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("CoralElevatorRaised", isRaised());
        SmartDashboard.putBoolean("CoralElevatorLowered", isLowered());
        SmartDashboard.putNumber("lowerVoltage",LowerLimitSwitch.getValue());
        SmartDashboard.putNumber("upperVoltage",UpperLimitSwitch.getValue());
    }
    private boolean isConstrained(double value, double min, double max){
        return value <= max && value >= min;
    }
}
