package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGripper;

public class HoldAlgae extends Command {
    private AlgaeGripper algaeGripper;

    public HoldAlgae(){
        this.algaeGripper = new AlgaeGripper();

        addRequirements(algaeGripper);
    }

    @Override
    public void initialize() {
        algaeGripper.rotateHold();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        algaeGripper.stop();
    }
}
