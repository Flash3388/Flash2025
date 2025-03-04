package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AlgaeGripper;

public class CollectAlgae extends Command {
    private final AlgaeGripper algaeGripper;
    private double startTime;
    private final double waitTime;
    private boolean finished;

    public CollectAlgae(AlgaeGripper algaeGripper, double waitTime){
        this.algaeGripper = algaeGripper;
        this.waitTime = waitTime;

        addRequirements(algaeGripper);
    }

    @Override
    public void initialize() {
       algaeGripper.rotateCollect();
       startTime = Timer.getTimestamp();
       finished = false;
    }

    @Override
    public void execute() {
        if (DriverStation.isAutonomous() && Timer.getTimestamp() - startTime >= waitTime) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (algaeGripper.hasAlgae() || finished);
    }

    @Override
    public void end(boolean interrupted) {
        algaeGripper.stop();
    }


}
