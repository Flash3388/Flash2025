package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class MoveToAngle extends Command {
    private final CoralArm coralArm;
    private final double angle;

    public MoveToAngle(CoralArm coralArm, double angle) {
        this.coralArm = coralArm;
        this.angle = angle;
        addRequirements(coralArm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        coralArm.simpleRetract(angle);

    }

    @Override
    public void end(boolean interrupted) {
        coralArm.stop();
    }

    @Override
    public boolean isFinished() {
        return coralArm.getPosition() == angle;
    }
}

}
