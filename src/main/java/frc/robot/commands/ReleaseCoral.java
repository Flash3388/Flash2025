package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGripper;

public class ReleaseCoral extends Command {
    public CoralGripper coralGripper;
    private double previousTime = -1;

    public ReleaseCoral(CoralGripper coralGripper) {
        this.coralGripper = coralGripper;
        addRequirements(coralGripper);
    }

    @Override
    public void initialize() {
        coralGripper.rotateRelease();
        previousTime = -1;
    }

    @Override
    public void execute() {
        if (!coralGripper.hasCoral()) {
            if (previousTime == -1) {
                previousTime = Timer.getFPGATimestamp();
            }
        }
    }

    @Override
    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp();
        return (previousTime != -1) && ((currentTime - previousTime) >= 0.5);
    }

    @Override
    public void end(boolean wasInterrupted) {
        coralGripper.stop();
    }
}
