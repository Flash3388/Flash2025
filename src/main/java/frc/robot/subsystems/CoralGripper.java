package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.ironmaple.simulation.drivesims.COTS;

public class CoralGripper extends SubsystemBase {


    private SparkMax motor;
    private DigitalInput limitSwitch;


    public CoralGripper() {
        motor = new SparkMax(RobotMap.MOTOR_IDENTIFIER, SparkLowLevel.MotorType.kBrushless);
        limitSwitch = new DigitalInput(1);
    }

    public boolean hasCoral() {
        return !limitSwitch.get();
    }

    public void rotateCollect() {
        motor.set(0.8);
    }

    public void rotateRelease() {
        motor.set(-0.5);
    }

    public void rotateHold() {
        motor.set(0.2);
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.getBoolean("Coral Gripper Limit", hasCoral());
    }
}


