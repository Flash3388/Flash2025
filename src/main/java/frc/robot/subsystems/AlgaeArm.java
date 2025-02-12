package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AlgaeArm extends SubsystemBase {
    private final DoubleSolenoid solenoid1;
    //private final DoubleSolenoid solenoid2;
    private final DigitalInput switchTop;
    private final DigitalInput switchBottom;

    public AlgaeArm(){
        this.solenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.ALGEA_ARM_FORWARD_PISTON_FORWARD_CHANNEL, RobotMap.ALGEA_ARM_FORWARD_PISTON_REVERSE_CHANNEL);
       // this.solenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.ALGEA_ARM_BACKWARD_PISTON_FORWARD_CHANNEL, RobotMap.ALGEA_ARM_BACKWARD_PISTON_REVERSE_CHANNEL);
        this.switchTop = new DigitalInput(RobotMap.ALGAE_ARM_SWITCH_TOP);
        this.switchBottom = new DigitalInput(RobotMap.ALGAE_ARM_SWITCH_BOTTOM);
    }

    public boolean isExtended(){
        return !(switchTop.get());
    }

    public boolean isRetracted(){
        return !(switchBottom.get());
    }

    public void extend(){
        solenoid1.set(DoubleSolenoid.Value.kForward);
        //solenoid2.set(DoubleSolenoid.Value.kForward);
    }

    public void retract(){
        solenoid1.set(DoubleSolenoid.Value.kReverse);
        //solenoid2.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop(){
        solenoid1.set(DoubleSolenoid.Value.kOff);
        //solenoid2.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    public void periodic(){
        //SmartDashboard.putBoolean("AlgaeArmExtended", isExtended());
        //SmartDashboard.putBoolean("AlgaeArmRetracted", isRetracted());
    }

}
