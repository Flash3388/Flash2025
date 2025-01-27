package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CoralGripper extends SubsystemBase {


    private TalonFX motor;
    private DutyCycleOut dutyCycleControl;
    private NeutralOut neutralControl;
    private DigitalInput limitSwitch;
    public static final double ROTATE_COLLECT = 0.8;
    public static final double ROTATE_RELEASE = -0.5;
    public static final double ROTATE_HOLD = 0.2;

    public CoralGripper() {
        motor = new TalonFX(RobotMap.CORAL_GRIPPER_MOTOR);
        motor.getConfigurator().apply(new TalonFXConfiguration());
        limitSwitch = new DigitalInput(RobotMap.CORAL_GRIPPER_LIMIT_SWITCH);

        dutyCycleControl = new DutyCycleOut(0);
        neutralControl = new NeutralOut();
    }

    public boolean hasCoral() {
        return !limitSwitch.get();
    }

    public double getVelocity(){
        return motor.getVelocity().getValue().in(motor.getVelocity().getValue().unit());
    }

    public final void rotateCollect() {
        motor.setControl(dutyCycleControl.withOutput(ROTATE_COLLECT));
    }

    public final void rotateRelease() {
        motor.setControl(dutyCycleControl.withOutput(ROTATE_RELEASE));
    }

    public final void rotateHold() {
        motor.setControl(dutyCycleControl.withOutput(ROTATE_HOLD));
    }

    public void stop() {
        motor.setControl(neutralControl);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("HasCoral", hasCoral());
    }
}


