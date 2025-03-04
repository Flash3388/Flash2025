package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGripper;
import java.time.Instant;

import java.util.Timer;

public class CollectCoral extends Command {
    public CoralGripper coralGripper;
    Instant previousTime = null;


    public CollectCoral(CoralGripper CoralGripper) {
        this.coralGripper = CoralGripper;

        addRequirements(coralGripper);
    }
    @Override
    public void initialize() {
        coralGripper.rotateCollect();
        previousTime = null;
    }

    @Override
    public void execute() {
        if(coralGripper.hasCoral()){
            coralGripper.rotateHold();
            if(previousTime == null){

                previousTime = Instant.now();
            }
        }

    }

    @Override
    public boolean isFinished() {
        Instant currentTime = Instant.now();
        return (previousTime!=null)&&(currentTime.isAfter(previousTime.plusMillis(300)));

    }

    @Override
    public void end(boolean wasInterrupted)  {
        coralGripper.stop();
    }
}
