package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;

public class ExtendedAlgaeArm extends Command {
    private final AlgaeArm algaeArm;

    public ExtendedAlgaeArm(AlgaeArm algaeArm){
        this.algaeArm = algaeArm;

        addRequirements(algaeArm);
    }

    @Override
    public void initialize() {
        algaeArm.extend();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return algaeArm.isExtended();
    }

    @Override
    public void end(boolean interrupted) {

    }

}
