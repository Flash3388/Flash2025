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
    private final AnalogInput switchTop;
    private final AnalogInput switchBottom;
    private final double MAX_UPPER_INPUT = 380;
    private final double MIN_UPPER_INPUT = 250;

    public AlgaeArm(){
        this.solenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.ALGEA_ARM_FORWARD_PISTON_FORWARD_CHANNEL, RobotMap.ALGEA_ARM_FORWARD_PISTON_REVERSE_CHANNEL);
        this.switchTop = new AnalogInput(RobotMap.ALGAE_ARM_SWITCH_TOP);
        this.switchBottom = new AnalogInput(RobotMap.ALGAE_ARM_SWITCH_BOTTOM);
    }

    public boolean isExtended(){
        return !isConstrained(switchTop.getValue(),MIN_UPPER_INPUT,MAX_UPPER_INPUT);
    }

    public boolean isRetracted(){
        return !isConstrained(switchBottom.getValue(),MIN_UPPER_INPUT,MAX_UPPER_INPUT);
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
//        SmartDashboard.putBoolean("AlgaeArmExtended", isExtended());
//        SmartDashboard.putBoolean("AlgaeArmRetracted", isRetracted());
//        SmartDashboard.putNumber("BottomValue",switchBottom.getValue());
//        SmartDashboard.putNumber("TopValue",switchTop.getValue());
    }

    private boolean isConstrained(double value, double min, double max){
        return value <= max && value >= min;
    }
}
