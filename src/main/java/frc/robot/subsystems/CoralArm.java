package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CoralArm extends SubsystemBase {

    private final SparkMax motor;
    private final AbsoluteEncoder absEncoder;
    private final SparkClosedLoopController controller;


    public CoralArm() {
        motor = new SparkMax(RobotMap.ARM_CORAL_MOTOR, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.absoluteEncoder
                .zeroOffset(RobotMap.ARM_CORAL_ZERO_OFFSET);
        config.limitSwitch
                .forwardLimitSwitchEnabled(true)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
        config.closedLoop
                .pid(RobotMap.ARM_CORAL_KP, RobotMap.ARM_CORAL_KI, RobotMap.ARM_CORAL_KD)
                .iZone(RobotMap.ARM_CORAL_IZONE)
                .outputRange(-1, 1)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(true);
        config.voltageCompensation(12);
        config.encoder
                .positionConversionFactor(1 / RobotMap.ARM_CORAL_GEAR_RATIO)
                .velocityConversionFactor(1 / RobotMap.ARM_CORAL_GEAR_RATIO);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        absEncoder = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();
    }

    public double getPositionDegrees() {
        return absEncoder.getPosition() * 360.0;
    }

    public double getPositionDegreesForFF() {
        return getPositionDegrees() - RobotMap.ARM_CORAL_FF_POS_OFFSET;
    }

    public boolean didReachPosition(double positionDegrees) {
        double currentPosition = getPositionDegrees();
        return MathUtil.isNear(positionDegrees, currentPosition, RobotMap.ARM_CORAL_TOLERANCE_POSITION_DEGREES) &&
                Math.abs(absEncoder.getVelocity()) < RobotMap.ARM_CORAL_TOLERANCE_VELOCITY_RPM;
    }

    public void setMoveToPosition(double positionDegrees) {
        double currentPositionForFF = getPositionDegreesForFF();
        double ff = RobotMap.ARM_CORAL_KF * Math.cos(Math.toRadians(currentPositionForFF));

        double positionRotations = positionDegrees / 360.0;
        controller.setReference(positionRotations, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    public void move(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
    }

}
