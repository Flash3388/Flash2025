package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGripper;

public class CollectAlgae extends Command {
    private AlgaeGripper algaeGripper;

    public CollectAlgae(AlgaeGripper algaeGripper){
        this.algaeGripper = algaeGripper;

        addRequirements(algaeGripper);
    }

    @Override
    public void initialize() {
       algaeGripper.rotateCollect();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return algaeGripper.hasAlgae();
    }

    @Override
    public void end(boolean interrupted) {

    }


}
