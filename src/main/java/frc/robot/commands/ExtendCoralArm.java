package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class ExtendCoralArm extends Command {
    private final CoralArm coralArm;
    private final double speed;
    public ExtendCoralArm(CoralArm coralArm, double speed){
        this.coralArm = coralArm;
        this.speed =speed;
        addRequirements(coralArm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        coralArm.simpleExtend(speed);

    }
    @Override
    public boolean isFinished() {

        return coralArm.isExtended();
    }
    @Override
    public void end(boolean interrupted) {
        coralArm.stop();
    }


}
