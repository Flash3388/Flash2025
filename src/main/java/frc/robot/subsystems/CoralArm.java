package frc.robot.subsystems;

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import java.util.concurrent.CancellationException;

public class CoralArm extends SubsystemBase {
    private SparkMax sparkMax ;

    private DigitalInput extendlimitswitch;
    private DigitalInput retractedLimitSwitch;
    public CoralArm(){
        sparkMax = new SparkMax(RobotMap.ARM_CORAL_MOTOR, SparkLowLevel.MotorType.kBrushless);
       this.extendlimitswitch = new DigitalInput(1);

        
    }
    public boolean isExtended(){
        return !extendlimitswitch.get();
    }
    public boolean isRetracted(){

        return !retractedLimitSwitch.get();
    }
    public void simpleExtend(double speed){
    sparkMax.set(speed);
    }
    public void simpleRetract(double speed){
    sparkMax.set(-speed);
    }
    public void stop(){
    sparkMax.stopMotor();
    }

    @Override
    public void periodic() {
        System.out.println("extended"+ isExtended());
        System.out.println("retracted"+ isRetracted());
    }

}
