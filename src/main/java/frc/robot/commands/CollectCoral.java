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

    public void initialize() {
        coralGripper.rotateCollect();
    }

    public void execute() {
    }

    public boolean isFinished() {
        return coralGripper.hasCoral();
    }

    public void end() {
    }
}
