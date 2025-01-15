package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class ExtendCoralArm extends Command {
    private CoralArm coralArm;
    public ExtendCoralArm(CoralArm coralArm){
        this.coralArm = coralArm;
        addRequirements(coralArm);
    }

    @Override
    public void initialize() {
        coralArm.extend();
    }

    @Override
    public void execute() {

    }
    @Override
    public boolean isFinished() {

        return coralArm.isExtended();
    }
    @Override
    public void end(boolean interrupted) {

    }


}
