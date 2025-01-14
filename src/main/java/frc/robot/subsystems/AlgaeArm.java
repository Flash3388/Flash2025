package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
    private DoubleSolenoid solenoidLeft;
    private DoubleSolenoid solenoidRight;
    private DigitalInput switchLeft;
    private DigitalInput switchRight;

    public AlgaeArm(DoubleSolenoid solenoidLeft, DoubleSolenoid solenoidRight, DigitalInput switchLeft, DigitalInput switchRight){
        this.solenoidLeft = solenoidLeft;
        this.solenoidRight = solenoidRight;
        this.switchLeft = switchLeft;
        this.switchRight = switchRight;
    }

    public boolean isExtended(){
        return !(switchLeft.get()) && !(switchRight.get());
    }

    public boolean isRetracted(){
        return switchLeft.get() && switchRight.get();
    }

    public void extend(){
        solenoidLeft.set(DoubleSolenoid.Value.kForward);
        solenoidRight.set(DoubleSolenoid.Value.kForward);
    }

    public void retract(){
        solenoidLeft.set(DoubleSolenoid.Value.kReverse);
        solenoidRight.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop(){
        solenoidLeft.set(DoubleSolenoid.Value.kOff);
        solenoidRight.set(DoubleSolenoid.Value.kOff);
    }

    public void periodic(){
        SmartDashboard.putBoolean("Extended", isExtended());
        SmartDashboard.putBoolean("Retracted", isRetracted());
    }
}
