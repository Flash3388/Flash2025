package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGripper;

public class CollectCoral extends Command {
    public CoralGripper coralGripper;

    public CollectCoral(CoralGripper CoralGripper) {
        this.coralGripper = CoralGripper;

        addRequirements(coralGripper);
    }
    @Override
    public void initialize() {
        coralGripper.rotateCollect();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return coralGripper.hasCoral();
    }

    @Override
    public void end(boolean wasInterrupted)  {
        coralGripper.stop();
    }
}
