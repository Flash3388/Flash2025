package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGripper;

public class HoldCoral extends Command {
    public CoralGripper coralGripper;


    public HoldCoral(CoralGripper coralGripper) {
        this.coralGripper = coralGripper;

        addRequirements(coralGripper);
    }

    @Override
    public void initialize() {
        coralGripper.rotateHold();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
