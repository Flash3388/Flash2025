package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class StopSwerve extends Command {
    private Swerve swerve;
    public StopSwerve(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {

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
