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

    private double targetPositionDegrees;
    private boolean hasNewTarget;
    private boolean didReachPosition;
    private boolean isHolding;

    public CoralArmCommand(CoralArm arm) {
        this.arm = arm;
        constraints = new TrapezoidProfile.Constraints(1000,200);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        isHolding = false;
    }
//TODO : stop the motion profiling when finished
    @Override
    public void execute() {
        if (hasNewTarget) {
            // reset to move to new angle
            if(!isHolding){
                arm.stop();
            }else {

                didReachPosition = false;
                hasNewTarget = false;
            }
            motionProfile = new TrapezoidProfile(constraints);
            motionProfileGoal = new TrapezoidProfile.State(targetPositionDegrees,0);
            motionProfileSetPoint = new TrapezoidProfile.State(arm.getPositionDegrees(),0);
        }
        motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);
        arm.setMoveToPosition(motionProfileSetPoint.position);
        //arm.setMoveToPosition(targetPositionDegrees);
        didReachPosition = arm.didReachPosition(targetPositionDegrees);
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
     }
}
