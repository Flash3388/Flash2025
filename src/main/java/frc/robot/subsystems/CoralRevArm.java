package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.ObjectInputFilter;

public class CoralRevArm extends SubsystemBase {
    private SparkMax armMotor;
    private SparkClosedLoopController controller;

    public CoralRevArm(){
        armMotor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
        controller = armMotor.getClosedLoopController();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
        armMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        config
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(0.25);
        armMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void moveToSetPoint(double setPoint){
        controller.setReference(setPoint, SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public void stop(){
        armMotor.stopMotor();
    }

    public boolean reachFinal(){
       AbsoluteEncoder absoluteEncoder = armMotor.getAbsoluteEncoder();
       if(absoluteEncoder.getPosition()<=0.05 || absoluteEncoder.getPosition()>=0.587){
           return true;
       }
       return false;
    }
    @Override
    public void periodic() {
        if(reachFinal()){
            stop();
        }
    }
}
