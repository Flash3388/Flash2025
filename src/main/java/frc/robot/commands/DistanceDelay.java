package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSystem;

public class DistanceDelay extends Command {

    private double distance;
    private final double finalDis =3;
    private int aprilTagId;

    private VisionSystem visionSystem;
    private Swerve swerve;
    public DistanceDelay(VisionSystem visionSystem,Swerve swerve , int aprilTagId) {
            this.aprilTagId = aprilTagId;
            this.visionSystem = visionSystem;
            this.swerve = swerve;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        distance = getAprilTagDistance(aprilTagId);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return distance<= finalDis;
    }

    public double getAprilTagDistance(int aprilTagID) {
        Pose2d aprilTagPose = visionSystem.getAprilTagPose(aprilTagID);
        return swerve.getPose().getTranslation().getDistance(aprilTagPose.getTranslation());
    }
}
