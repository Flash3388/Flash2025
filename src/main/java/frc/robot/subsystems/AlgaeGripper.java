package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class AlgaeGripper extends SubsystemBase {
     private static final double COLLECT_SPEED = -0.8;
     private static final double RELEASE_SPEED = 0.8;
     private static final double HOLD_SPEED = 0.8;

     private final SparkMax motor;
     private final SparkMax motor2;
     private final RelativeEncoder encoder;
     private final DigitalInput digitalInput;

     public AlgaeGripper(){
          this.motor = new SparkMax(RobotMap.ALGAE_GRIPPER_MOTOR1, SparkLowLevel.MotorType.kBrushless);

          SparkMaxConfig config = new SparkMaxConfig();
          config.idleMode(SparkBaseConfig.IdleMode.kBrake);
          motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

          encoder = this.motor.getEncoder();

          this.motor2 = new SparkMax(RobotMap.ALGAE_GRIPPER_MOTOR2, SparkLowLevel.MotorType.kBrushless);
          config = new SparkMaxConfig();
          config.idleMode(SparkBaseConfig.IdleMode.kBrake);
          motor2.configure(config,SparkBase.ResetMode.kNoResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters);

          this.digitalInput = new DigitalInput(RobotMap.ALGAE_GRIPPER_DIGITALINPUT);
     }

     public boolean hasAlgae(){
          return !digitalInput.get();
     }

     public double getVelocityRpm(){
          return encoder.getVelocity();
     }

     public void rotateCollect(){
          motor.set(COLLECT_SPEED);
          motor2.set(-COLLECT_SPEED);
     }

     public void rotateRelease(){
          motor.set(RELEASE_SPEED);
          motor2.set(-RELEASE_SPEED);
     }

     public void rotateHold(){
          motor.set(HOLD_SPEED);
          motor2.set(-HOLD_SPEED);
     }

     public void stop(){
          motor.stopMotor();
          motor2.stopMotor();
     }

     @Override
     public void periodic(){
        //  SmartDashboard.putBoolean("HasAlgae", hasAlgae());
     }
}
