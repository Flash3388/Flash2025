package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CoralGripper extends SubsystemBase {

    public static final double ROTATE_COLLECT = -0.8;
    public static final double ROTATE_RELEASE = 0.5;
    public static final double ROTATE_HOLD = -0.08;

    private final TalonFX motor;
    private final DigitalInput limitSwitch;
    private final DutyCycleOut dutyCycleControl;
    private final NeutralOut neutralControl;
    private final StatusSignal<AngularVelocity> velocitySignal;

    public CoralGripper() {
        motor = new TalonFX(RobotMap.CORAL_GRIPPER_MOTOR);
        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.setNeutralMode(NeutralModeValue.Brake);
        limitSwitch = new DigitalInput(RobotMap.CORAL_GRIPPER_LIMIT_SWITCH);

        velocitySignal = motor.getVelocity();

        dutyCycleControl = new DutyCycleOut(0);
        neutralControl = new NeutralOut();
    }

    public boolean hasCoral() {
        return !limitSwitch.get();
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
        BaseStatusSignal.refreshAll(velocitySignal);
    }
}


