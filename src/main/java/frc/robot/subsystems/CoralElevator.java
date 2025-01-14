package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;

import static com.sun.tools.sjavac.Util.set;

public class CoralElevator {

    private Solenoid DoubleSolenoid;
    private Solenoid DoubleSolenoid2;
    private DigitalInput UpperLimitSwitch;
    private DigitalInput LowerLimitSwitch;

    public boolean isRaised(){
        if (UpperLimitSwitch.get() ){
            return true;
        }
        return false;
    }

    public boolean isLowered(){
        if (LowerLimitSwitch.get()){
            return true;
        }
        return false;
    }

    public void raise(){

    }

}
