package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArm extends SubsystemBase {
    private DoubleSolenoid piston;
    private DoubleSolenoid piston2;
    private DigitalInput extendlimitswitch;
    private DigitalInput retractlimitswitch;
    public CoralArm(){
        this.piston = new DoubleSolenoid(PneumaticsModuleType.);
        this.piston2 = new DoubleSolenoid(PneumaticsModuleType.);
        this.extendlimitswitch = new DigitalInput(1);
        this.retractlimitswitch = new DigitalInput(2);

        
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
