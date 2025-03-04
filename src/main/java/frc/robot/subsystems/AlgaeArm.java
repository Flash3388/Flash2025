package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AlgaeArm extends SubsystemBase {

    private final DoubleSolenoid solenoid1;


    public AlgaeArm(){
        this.solenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.ALGEA_ARM_FORWARD_PISTON_FORWARD_CHANNEL, RobotMap.ALGEA_ARM_FORWARD_PISTON_REVERSE_CHANNEL);
    }

    public void extend(){
        solenoid1.set(DoubleSolenoid.Value.kForward);
    }

    public void retract(){
        solenoid1.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop(){
        solenoid1.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    public void periodic(){
    }

    private boolean isConstrained(double value, double min, double max){
        return value <= max && value >= min;
    }
}
