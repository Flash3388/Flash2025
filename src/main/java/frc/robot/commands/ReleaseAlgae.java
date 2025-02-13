package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGripper;

public class ReleaseAlgae extends Command {
    private final AlgaeGripper algaeGripper;
    private double previousTime = -1;

    public ReleaseAlgae(AlgaeGripper algaeGripper){
        this.algaeGripper = algaeGripper;
        addRequirements(algaeGripper);
    }

    @Override
    public void initialize() {
        algaeGripper.rotateRelease();
        previousTime = -1;
    }

    @Override
    public void execute() {
        if (!algaeGripper.hasAlgae()) {
            algaeGripper.rotateHold();
            if (previousTime == -1) {
                previousTime = Timer.getFPGATimestamp();
            }
        }
    }

    @Override
    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp();
        return (previousTime != -1) && ((currentTime - previousTime) >= 0.7);
    }

    @Override
    public void end(boolean interrupted) {
        algaeGripper.stop();
    }
}
