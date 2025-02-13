package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGripper;

public class CollectCoral extends Command {
    public CoralGripper coralGripper;
    private double previousTime = -1;

    public CollectCoral(CoralGripper CoralGripper) {
        this.coralGripper = CoralGripper;
        addRequirements(coralGripper);
    }

    @Override
    public void initialize() {
        coralGripper.rotateCollect();
        previousTime = -1;
    }

    @Override
    public void execute() {
        if (coralGripper.hasCoral()) {
            coralGripper.rotateHold();
            if (previousTime == -1) {
                previousTime = Timer.getFPGATimestamp();
            }
        }
    }

    @Override
    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp();
        return (previousTime != -1) && ((currentTime - previousTime) >= 0.3);
    }

    @Override
    public void end(boolean wasInterrupted) {
        coralGripper.stop();
    }
}
