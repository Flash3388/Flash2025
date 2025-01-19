package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import java.util.concurrent.CancellationException;

public class CoralArm extends SubsystemBase {
    private SparkMax sparkMax ;
    private PIDController pid;
    private DigitalInput extendlimitswitch;
    private DigitalInput retractedLimitSwitch;
    private AbsoluteEncoder encoder;
    public CoralArm(){
        sparkMax = new SparkMax(RobotMap.ARM_CORAL_MOTOR, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        final int START_PULSE_US = 1;
        final int END_PULSE_US = 1024;
        final double ZERO_OFFSET = 0.2;

        config.absoluteEncoder.startPulseUs(START_PULSE_US).endPulseUs(END_PULSE_US).zeroOffset(ZERO_OFFSET);
        encoder = sparkMax.getAbsoluteEncoder();
        pid = new PIDController(RobotMap.KP,RobotMap.KI,RobotMap.KD);
        pid.setIZone(RobotMap.IZONE);
        pid.setTolerance(RobotMap.TOLORANCE);
       this.extendlimitswitch = new DigitalInput(1);

        
    }

    public void moveToAngle(double angle){
        // angle offsets logics
        double speed = pid.calculate(getPosition(),angle);
        // angle offsets logics
        sparkMax.set(speed);
    }
    public double getPosition(){
        return encoder.getPosition()*360;
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
