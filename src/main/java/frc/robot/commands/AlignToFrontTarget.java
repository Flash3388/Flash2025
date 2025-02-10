package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSystem;

import java.util.ArrayList;
import java.util.List;


public class AlignToFrontTarget extends Command {

    private VisionSystem vision;
    private Swerve swerve;

    private List<Waypoint> waypoints;
    private PathPlannerPath path;
    private Pose2d targetPose;

    public AlignToFrontTarget(VisionSystem visionSystem, Swerve swerve) {
        vision = visionSystem;
        this.swerve = swerve;


        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        int id = vision.frontGetTargetId();
        targetPose = vision.getIdPose(id).get();
        Pose2d pose = new Pose2d(targetPose.getX()+0.1, targetPose.getY()+0.1, targetPose.getRotation());
        Pose2d leftPose = new Pose2d(13.73,4.75,new Rotation2d(-120));

        swerve.rotate(()-> vision.getMovingAngle(id));

        waypoints = PathPlannerPath.waypointsFromPoses(
                swerve.getPose(),
                leftPose
        );
        PathConstraints constraints = new PathConstraints(0.5,0.5,Math.PI*2,Math.PI);

        path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0,new Rotation2d(vision.getMovingAngle(id)))
        );
        path.preventFlipping = true;
        AutoBuilder.followPath(path).schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return targetPose != null && swerve.isNear(targetPose);
    }

}
