package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGripper;

import java.time.Instant;

public class ReleaseCoral extends Command {
    public CoralGripper coralGripper;
    Instant previousTime = null;

    public ReleaseCoral(CoralGripper coralGripper) {
        this.coralGripper = coralGripper;

        addRequirements(coralGripper);
    }

    @Override
    public void initialize() {
        coralGripper.rotateRelease();
        previousTime = null;
    }

    @Override
    public void execute() {
        if(!coralGripper.hasCoral()){
            if(previousTime == null){
                previousTime = Instant.now();
            }
        }
    }

    @Override
    public boolean isFinished() {
        Instant currentTime = Instant.now();
        return (previousTime!=null)&&(currentTime.isAfter(previousTime.plusMillis(500)));
    }

    @Override
    public void end(boolean wasInterrupted) {
        coralGripper.stop();
    }
}
