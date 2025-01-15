package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class RetractCoralArm extends Command {
    private CoralArm coralArm;

    public RetractCoralArm(CoralArm coralArm) {
        this.coralArm = coralArm;
    }

    @Override
    public void initialize() {
        coralArm.retract();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return coralArm.isRetracted();
    }
}
