package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CoralElevator extends SubsystemBase {

    private final DoubleSolenoid piston1;
    private final DoubleSolenoid piston2;
    private final DigitalInput UpperLimitSwitch;
    private final DigitalInput LowerLimitSwitch;

    public CoralElevator() {
        piston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.CORAL_ELEVATOR_PISTON_FORWARD_CHANNEL, RobotMap.CORAL_ELEVATOR_PISTON_REVERSE_CHANNEL);
        piston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.CORAL_ELEVATOR_PISTON_FORWARD_CHANNEL, RobotMap.CORAL_ELEVATOR_PISTON_REVERSE_CHANNEL);
        UpperLimitSwitch = new DigitalInput(RobotMap.CORAL_ELEVATOR_UPPER_LIMIT_SWITCH);
        LowerLimitSwitch = new DigitalInput(RobotMap.CORAL_ELEVATOR_LOWER_LIMIT_SWITCH);
    }

    public boolean isRaised() {
        return !UpperLimitSwitch.get();
    }

    public boolean isLowered() {
        return !LowerLimitSwitch.get();
    }

    public void raise() {
        piston1.set(DoubleSolenoid.Value.kForward);
        piston2.set(DoubleSolenoid.Value.kForward);
    }

    public void lower() {
        piston1.set(DoubleSolenoid.Value.kReverse);
        piston2.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop() {
        piston1.set(DoubleSolenoid.Value.kOff);
        piston2.set(DoubleSolenoid.Value.kOff);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator Raised: " , isRaised());
        SmartDashboard.putBoolean("Elevator Lowered: " , isLowered());
    }
}
