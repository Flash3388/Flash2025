package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGripper;

public class ReleaseCoral extends Command {
    public CoralGripper coralGripper;

    public ReleaseCoral(CoralGripper coralGripper) {
        this.coralGripper = coralGripper;

        addRequirements(coralGripper);
    }

    public void initialize() {
        coralGripper.rotateRelease();
    }

    public void execute() {
    }

    public boolean isFinished() {
        return !coralGripper.hasCoral();
    }

    public void end() {
    }
}
