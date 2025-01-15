package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArm extends SubsystemBase {
    private final DoubleSolenoid piston;
    private final DoubleSolenoid piston2;
    private final DigitalInput extendlimitswitch;
    private final DigitalInput retractlimitswitch;
    public CoralArm(DoubleSolenoid piston, DoubleSolenoid piston2,DigitalInput extendlimitswitch, DigitalInput retractlimitswitch){
        this.piston = piston;
        this.piston2 = piston2;
        this.extendlimitswitch = extendlimitswitch;
        this.retractlimitswitch = retractlimitswitch;

        
    }
    public boolean isExtended(){
        return !extendlimitswitch.get();
    }
    public boolean isRetracted(){

        return !retractlimitswitch.get();
    }
    public void extend(){
        piston.set(DoubleSolenoid.Value.kForward);
        piston2.set(DoubleSolenoid.Value.kForward);

    }
    public void retract(){
        piston.set(DoubleSolenoid.Value.kReverse);
        piston2.set(DoubleSolenoid.Value.kReverse);

    }
    public void stop(){
        piston.set(DoubleSolenoid.Value.kOff);
        piston2.set(DoubleSolenoid.Value.kOff);

    }

    @Override
    public void periodic() {
        System.out.println("extended"+ isExtended());
        System.out.println("retracted"+ isRetracted());
    }

}
