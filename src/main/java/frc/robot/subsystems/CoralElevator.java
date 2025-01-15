package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CoralElevator extends SubsystemBase {

    private DoubleSolenoid piston1;
    private DoubleSolenoid piston2;
    private DigitalInput UpperLimitSwitch;
    private DigitalInput LowerLimitSwitch;

    public CoralElevator() {
        piston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PISTON_FORWARD_CHANNEL, RobotMap.PISTON_REVERSE_CHANNEL);
        piston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.PISTON_FORWARD_CHANNEL, RobotMap.PISTON_REVERSE_CHANNEL);
        UpperLimitSwitch = new DigitalInput(1);
        LowerLimitSwitch = new DigitalInput(2);
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
        System.out.println("Elevator Raised: " + isRaised());
        System.out.println("Elevator Lowered: " + isLowered());
    }
}
