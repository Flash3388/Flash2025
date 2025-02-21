package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;

public class StopSwerveCommand extends Command {
    private Swerve swerve;
    public StopSwerveCommand(Swerve swerveSystem) {
        swerve = swerveSystem;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancel(swerve.getCurrentCommand());

        swerve.stop();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
