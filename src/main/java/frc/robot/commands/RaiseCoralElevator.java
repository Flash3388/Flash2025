package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator;

public class RaiseCoralElevator extends Command {

    private CoralElevator coralElevator;

    public RaiseCoralElevator(CoralElevator coralElevator) {
        this.coralElevator = coralElevator;

        addRequirements(coralElevator);
    }

    @Override
    public void initialize() {
        coralElevator.raise();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return coralElevator.isRaised();
    }

}
