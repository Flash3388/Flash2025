package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class RetractCoralArm extends Command {
    private final CoralArm coralArm;
    private final double speed;

    public RetractCoralArm(CoralArm coralArm, double speed) {
        this.coralArm = coralArm;
        this.speed = speed;
        addRequirements(coralArm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        coralArm.simpleRetract(speed);

    }

    @Override
    public void end(boolean interrupted) {
        coralArm.stop();
    }

    @Override
    public boolean isFinished() {
        return coralArm.isRetracted();
    }
}
