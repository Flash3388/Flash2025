package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGripper;

import java.time.Instant;

public class ReleaseAlgae extends Command {
    private final AlgaeGripper algaeGripper;
    Instant previousTime = null;

    public ReleaseAlgae(AlgaeGripper algaeGripper){
        this.algaeGripper = algaeGripper;

        addRequirements(algaeGripper);
    }

    @Override
    public void initialize() {
        algaeGripper.rotateRelease();
        previousTime = null;
    }

    @Override
    public void execute() {
        if(!algaeGripper.hasAlgae()){
            algaeGripper.rotateHold();
            if(previousTime == null){
                previousTime = Instant.now();
            }
        }

    }

    @Override
    public boolean isFinished() {
        Instant currentTime = Instant.now();
        return (previousTime!=null)&&currentTime.isAfter(previousTime.plusMillis(700));
    }

    @Override
    public void end(boolean interrupted) {
        algaeGripper.stop();
    }
}
