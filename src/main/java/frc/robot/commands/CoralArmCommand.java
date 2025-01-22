package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class CoralArmCommand extends Command {

    private final CoralArm arm;

    private double targetPositionDegrees;
    private boolean hasNewTarget;
    private boolean didReachPosition;

    public CoralArmCommand(CoralArm arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (hasNewTarget) {
            // reset to move to new angle

            didReachPosition = false;
            hasNewTarget = false;
        }

        arm.setMoveToPosition(targetPositionDegrees);
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
    }

    public boolean didReachTargetPosition() {
        if (hasNewTarget) {
            return false;
        }

        return didReachPosition;
    }
}
