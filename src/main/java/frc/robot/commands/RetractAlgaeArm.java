package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;

public class RetractAlgaeArm extends Command {
    private final AlgaeArm algaeArm;

    public RetractAlgaeArm(AlgaeArm algaeArm){
        this.algaeArm = algaeArm;

        addRequirements(algaeArm);
    }

    @Override
    public void initialize() {
        algaeArm.retract();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true; // algaeArm.isRetracted();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
