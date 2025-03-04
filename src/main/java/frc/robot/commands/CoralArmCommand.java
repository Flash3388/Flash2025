package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.CoralArm;

public class CoralArmCommand extends Command {

    private static final double MAX_VELOCITY_DEGREES_PER_SEC = 185;
    private static final double MAX_ACCELERATION_DEGREES_PER_SEC_PER_SEC = 175 * 2;

    private final CoralArm arm;
    private final TrapezoidProfile.Constraints constraints;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;

    private double targetPositionDegrees;
    private boolean hasNewTarget;
    private boolean didReachPosition;
    private boolean isHolding;
    private boolean wasDisabled;

    public CoralArmCommand(CoralArm arm) {
        this.arm = arm;
        constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY_DEGREES_PER_SEC, MAX_ACCELERATION_DEGREES_PER_SEC_PER_SEC);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        isHolding = false;
        hasNewTarget = true;
        wasDisabled = DriverStation.isDisabled();
    }

    @Override
    public void execute() {
        if (DriverStation.isDisabled()) {
            if (!wasDisabled) {
                hasNewTarget = true;
                isHolding = false;
                wasDisabled = true;
            }
        } else if (wasDisabled) {
            wasDisabled = false;
        }

        if (hasNewTarget) {
            // reset to move to new angle
            hasNewTarget = false;
            didReachPosition = false;

            //SmartDashboard.putBoolean("ArmCommandReached", false);

            if (isHolding) {
                motionProfile = new TrapezoidProfile(constraints);
                motionProfileGoal = new TrapezoidProfile.State(targetPositionDegrees,0);
                motionProfileSetPoint = new TrapezoidProfile.State(arm.getPositionDegrees(),0);

               //SmartDashboard.putNumber("ArmCommandTarget", targetPositionDegrees);
            } else {
                arm.stop();
                //SmartDashboard.putNumber("ArmCommandTarget", -1);
            }
        }

        if (!isHolding) {
            return;
        }

        if (!didReachPosition && arm.didReachPosition(targetPositionDegrees)) {
            didReachPosition = true;
            //SmartDashboard.putBoolean("ArmCommandReached", true);
        }

        if (didReachPosition) {
            arm.setMoveToPosition(targetPositionDegrees);
        } else {
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);
            arm.setMoveToPosition(motionProfileSetPoint.position);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    public void setNewTargetPosition(double positionDegrees) {
        this.targetPositionDegrees = positionDegrees;
        this.hasNewTarget = true;
        isHolding = true;
    }

    public boolean didReachTargetPosition() {
        if (hasNewTarget) {
            return false;
        }

        return didReachPosition;
    }

    public void stopHolding(){
        isHolding = false;
        hasNewTarget = true;
    }
}
