package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class CoralArmCommand extends Command {

    private static final double MAX_VELOCITY_DEGREES_PER_SEC = 2000;
    private static final double MAX_ACCELERATION_DEGREES_PER_SEC_PER_SEC = 20;

    private final CoralArm arm;
    private final TrapezoidProfile.Constraints constraints;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;

    private double targetPositionDegrees;
    private boolean hasNewTarget;
    private boolean didReachPosition;
    private boolean isHolding;

    public CoralArmCommand(CoralArm arm) {
        this.arm = arm;
        constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY_DEGREES_PER_SEC, MAX_ACCELERATION_DEGREES_PER_SEC_PER_SEC);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        isHolding = false;
        hasNewTarget = true;
    }
    @Override
    public void execute() {

        SmartDashboard.putBoolean("didReachPosArm",arm.didReachPosition(targetPositionDegrees));
        if (hasNewTarget) {
            // reset to move to new angle
            hasNewTarget = false;
            didReachPosition = false;

            if (isHolding) {
                motionProfile = new TrapezoidProfile(constraints);
                motionProfileGoal = new TrapezoidProfile.State(targetPositionDegrees,0);
                motionProfileSetPoint = new TrapezoidProfile.State(arm.getPositionDegrees(),0);
            } else {
                arm.stop();
            }
        }

        if (!isHolding) {
            return;
        }

        if (!didReachPosition && arm.didReachPosition(targetPositionDegrees)) {
            didReachPosition = true;
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
