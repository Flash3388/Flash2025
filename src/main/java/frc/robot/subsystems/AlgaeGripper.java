package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGripper extends SubsystemBase {
     private SparkMax motor;
     private DigitalInput digitalInput;
     private static final double COLLECT_SPEED = 0.8;
     private static final double RELEASE_SPEED = -0.5;
     private static final double HOLD_SPEED = 0.2;

     public AlgaeGripper(SparkMax motor, DigitalInput digitalInput){
          this.motor = motor;
          this.digitalInput = digitalInput;
     }

     public boolean hasAlgae(){
          return !digitalInput.get();
     }

     public void rotateCollect(){
          motor.set(COLLECT_SPEED);
     }

     public void rotateRelease(){
          motor.set(RELEASE_SPEED);
     }

     public void rotateHold(){
          motor.set(HOLD_SPEED);
     }

     public void stop(){
          motor.stopMotor();
     }

     public void periodic(){
          SmartDashboard.putBoolean("HasAlgae", hasAlgae());
     }
}
