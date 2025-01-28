package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AlgaeGripper extends SubsystemBase {
     private static final double COLLECT_SPEED = 0.8;
     private static final double RELEASE_SPEED = -0.5;
     private static final double HOLD_SPEED = 0.2;
     private final SparkMax motor;
     private final RelativeEncoder encoder;
     private final DigitalInput digitalInput;


     public AlgaeGripper(){
          this.motor = new SparkMax(RobotMap.ALGAE_GRIPPER_MOTOR, SparkLowLevel.MotorType.kBrushless);

          encoder = this.motor.getEncoder();

          this.digitalInput = new DigitalInput(RobotMap.ALGAE_GRIPPER_DIGITALINPUT);

          SparkMaxConfig config = new SparkMaxConfig();
          motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
     }

     public boolean hasAlgae(){
          return !digitalInput.get();
     }

     public double getVelocityRpm(){
          return encoder.getVelocity();
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

     @Override
     public void periodic(){
          SmartDashboard.putBoolean("HasAlgae", hasAlgae());
     }
}
