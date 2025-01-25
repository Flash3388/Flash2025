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
    private final RelativeEncoder relEncoder;
    private final SparkClosedLoopController controller;
    private final SparkLimitSwitch forwardLimitSwitch;
    private final SparkLimitSwitch reverseLimitSwitch;


    public CoralArm() {
        motor = new SparkMax(RobotMap.ARM_CORAL_MOTOR, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.absoluteEncoder
                .startPulseUs(RobotMap.ARM_CORAL_START_PULSE_US)
                .endPulseUs(RobotMap.ARM_CORAL_END_PULSE_US)
                .zeroOffset(RobotMap.ARM_CORAL_ZERO_OFFSET);
        config.limitSwitch
                .forwardLimitSwitchEnabled(true)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
        config.softLimit
                .forwardSoftLimit(180)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(0)
                .reverseSoftLimitEnabled(true);
        config.closedLoop
                .pid(RobotMap.ARM_CORAL_KP, RobotMap.ARM_CORAL_KI, RobotMap.ARM_CORAL_KD)
                .iZone(RobotMap.ARM_CORAL_IZONE)
                .outputRange(-1, 1)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1 / RobotMap.ARM_CORAL_GEAR_RATIO)
                .velocityConversionFactor(1 / RobotMap.ARM_CORAL_GEAR_RATIO);
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        relEncoder =  motor.getEncoder();
        absEncoder = motor.getAbsoluteEncoder();
        relEncoder.setPosition(absEncoder.getPosition());


        controller = motor.getClosedLoopController();
        forwardLimitSwitch = motor.getForwardLimitSwitch();
        reverseLimitSwitch = motor.getReverseLimitSwitch();

    }

    public double getPositionDegrees() {
        return relEncoder.getPosition() * 360;
    }

    public boolean didReachPosition(double positionDegrees) {
        double currentPosition = getPositionDegrees();
        return MathUtil.isNear(positionDegrees, currentPosition, RobotMap.ARM_CORAL_TOLERANCE_POSITION_DEGREES) &&
                Math.abs(relEncoder.getVelocity()) < RobotMap.ARM_CORAL_TOLERANCE_VELOCITY_RPM;
    }

    public boolean isAtForwardLimit() {
        return forwardLimitSwitch.isPressed();
    }

    public boolean isAtReverseLimit() {
        return reverseLimitSwitch.isPressed();
    }

    public void setMoveToPosition(double positionDegrees) {
        double currentPosition = getPositionDegrees();
        double ff = RobotMap.ARM_CORAL_KF * Math.cos(Math.toRadians(currentPosition));

        double positionRotations = positionDegrees / 360.0;
        controller.setReference(positionRotations, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("CoralArmAtForwardLimit", isAtForwardLimit());
        SmartDashboard.putBoolean("CoralArmAtReverseLimit", isAtReverseLimit());
        SmartDashboard.putNumber("CoralArmPositionRelative", getPositionDegrees());
        SmartDashboard.putNumber("CoralArmPositionAbsolute",absEncoder.getPosition());
    }

}
