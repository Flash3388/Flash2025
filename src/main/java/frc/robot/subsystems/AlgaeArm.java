package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AlgaeArm extends SubsystemBase {
    private DoubleSolenoid solenoidForward;
    private DoubleSolenoid solenoidBackward;
    private DigitalInput switchTop;
    private DigitalInput switchBottom;

    public AlgaeArm(){
        this.solenoidForward = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.FORWARD_PISTON_FORWARD_CHANNEL, RobotMap.FORWARD_PISTON_REVERSE_CHANI);
        this.solenoidBackward = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.BACKWARD_PISTON_FORWARD_CHANNEL, RobotMap.BACKWARD_PISTON_REVERSE_CHANI);
        this.switchTop = new DigitalInput(RobotMap.ALGAE_ARM_TOP);
        this.switchBottom = new DigitalInput(RobotMap.ALGAE_ARM_BOTTOM);
    }

    public boolean isExtended(){
        return !(switchTop.get()) && !(switchBottom.get());
    }

    public boolean isRetracted(){
        return switchTop.get() && switchBottom.get();
    }

    public void extend(){
        solenoidForward.set(DoubleSolenoid.Value.kForward);
        solenoidBackward.set(DoubleSolenoid.Value.kForward);
    }

    public void retract(){
        solenoidForward.set(DoubleSolenoid.Value.kReverse);
        solenoidBackward.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop(){
        solenoidForward.set(DoubleSolenoid.Value.kOff);
        solenoidBackward.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("AlgaeArmExtended", isExtended());
        SmartDashboard.putBoolean("AlgaeArmRetracted", isRetracted());
    }

}
