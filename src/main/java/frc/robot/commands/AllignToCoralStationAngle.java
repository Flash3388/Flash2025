package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSystem;

public class AllignToCoralStationAngle extends Command {
    private VisionSystem visionSystem;
    private Swerve swerve;
    private double setPoint;
    private double currentAngle;


    public AllignToCoralStationAngle(VisionSystem vision, Swerve swerveSystem) {
        visionSystem =vision;
        swerve = swerveSystem;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d robotPose =swerve.getPose();
        setPoint = visionSystem.angleToCoralStation(robotPose);
        currentAngle = robotPose.getRotation().getDegrees();
        swerve.driveA( () -> 0, () -> 0, () -> setPoint-currentAngle);
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
        return MathUtil.isNear(setPoint,currentAngle,1);
    }
}
