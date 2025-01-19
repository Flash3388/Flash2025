package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator;

public class LowerCoralElevator extends Command {

    private final CoralElevator coralElevator;

    public LowerCoralElevator(CoralElevator coralElevator) {
        this.coralElevator = coralElevator;
        addRequirements(coralElevator);
    }

    @Override
    public void initialize() {
        coralElevator.lower();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return coralElevator.isLowered();
    }


}
