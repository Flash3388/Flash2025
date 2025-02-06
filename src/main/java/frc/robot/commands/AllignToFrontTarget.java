package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSystem;

public class AllignToFrontTarget extends Command {

    private Swerve swerve;
    private VisionSystem vision;

    private double targetAngle;
    private double currentAngle;
    private double finalAngle;

    public AllignToFrontTarget(Swerve swerveSystem, VisionSystem visionSystem) {
        vision =visionSystem;
        swerve = swerveSystem;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        targetAngle = vision.frontTargetAngle();
        currentAngle = swerve.getPose().getRotation().getDegrees();
        finalAngle = 180 - (targetAngle - currentAngle);
        swerve.driveA(
                () ->0,
                () ->0,
                () -> finalAngle);
    }

    @Override
    public void execute() {
        currentAngle = swerve.getPose().getRotation().getDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(finalAngle,currentAngle,1.5);
    }
}
