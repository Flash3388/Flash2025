package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class CoralArmCommand extends Command {

    private final CoralArm arm;
    private final TrapezoidProfile.Constraints constraints;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;

    private static final double MAX_VELOCITY = 1000;
    private static final double MAX_ACCELERATION = 200;

    private double targetPositionDegrees;
    private boolean hasNewTarget;
    private boolean didReachPosition;
    private boolean isHolding;

    public CoralArmCommand(CoralArm arm) {
        this.arm = arm;
        constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        isHolding = false;
        hasNewTarget = true;
    }
    @Override
    public void execute() {
        if (hasNewTarget) {
            // reset to move to new angle

        }
        if(isHolding){
            arm.setMoveToPosition(targetPositionDegrees);
        }
        didReachPosition = false;
        hasNewTarget = false;

        motionProfile = new TrapezoidProfile(constraints);


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
