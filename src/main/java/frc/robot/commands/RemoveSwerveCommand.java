package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;

public class RemoveSwerveCommand extends Command {
    Swerve swerve;
    public RemoveSwerveCommand(Swerve swerve1) {
        swerve = swerve1;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().removeComposedCommand(swerve.getCurrentCommand());
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
