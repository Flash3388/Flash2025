package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import org.dyn4j.geometry.Vector2;

import java.util.Optional;

public class VisionSystem extends SubsystemBase {

    private static final String LL_NAME_BACK = "limelight-back";
    private static final String LL_NANE_FRONT = "limelight-front";

    private final Limelight limelightBack;
    private final Limelight limelightFront;
    private final AprilTagFieldLayout layout;

    public VisionSystem() {
        limelightBack = new Limelight(LL_NAME_BACK);
        limelightFront = new Limelight(LL_NANE_FRONT);
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public Optional<LimelightHelpers.PoseEstimate> getRobotPoseEstimate() {
        if (limelightFront.hasDetectedTarget() && limelightFront.getDistanceToTarget() <= 2.7) {
            return Optional.of(limelightFront.getPoseEstimate());
        } /*else if (limelightBack.hasDetectedTarget() && limelightBack.getDistanceToTarget() <= 2.5) {
            //return Optional.of(limelightBack.getPoseEstimate());
        }*/

        return Optional.empty();
    }

    public Pose2d getAprilTagPose(int id) {
        return layout.getTagPose(id).orElseThrow().toPose2d();
    }

    public void changePipeline(int id) {
        limelightFront.changePipeline(id);
        limelightBack.changePipeline(id);
    }

    public Pose2d getPoseForReefStand(int id, ReefStandRow row) {
        Pose3d pose3d = layout.getTagPose(id).orElseThrow();
        Pose2d pose = pose3d.toPose2d();

        Pose2d calculatedPose = calcPoseTwoSides(pose, RobotMap.OFFSET_ON_STAND, RobotMap.OFFSET_REEF, row == ReefStandRow.LEFT);
        return new Pose2d(calculatedPose.getX(), calculatedPose.getY(), calculatedPose.getRotation());
    }

    public Pose2d getPoseForFeeder(int id, FeederSide side) {
        Pose3d pose3d = layout.getTagPose(id).orElseThrow();
        Pose2d pose = pose3d.toPose2d();

        Pose2d calculatedPose;
        switch (side) {
            case LEFT:
            case RIGHT:
                calculatedPose = calcPoseTwoSides(pose, RobotMap.OFFSET_ON_STAND, RobotMap.OFFSET_FEEDER, side == FeederSide.LEFT);
                break;
            case CENTER:
                calculatedPose = calcPoseCenter(pose, RobotMap.OFFSET_FEEDER);
                break;
            default:
                throw new AssertionError();
        }

        return new Pose2d(calculatedPose.getX(), calculatedPose.getY(), calculatedPose.getRotation());
    }

    public Pose2d getPoseToProcessor(int id) {
        Pose3d pose3d = layout.getTagPose(id).orElseThrow();
        Pose2d pose = pose3d.toPose2d();

        Pose2d calculatedPose = calcPoseCenter(pose, RobotMap.OFFSET_PROCESSOR);
        return new Pose2d(calculatedPose.getX(), calculatedPose.getY(), calculatedPose.getRotation());
    }

    public Pose2d calcPoseTwoSides(Pose2d pose, double d1, double d2, boolean isLeft) {
        double alpha = pose.getRotation().getDegrees();
        double beta = isLeft ? alpha - 90 : alpha + 90;

        Vector2 start = new Vector2(pose.getX(), pose.getY());
        Vector2 down = Vector2.create(d1, Math.toRadians(beta));
        Vector2 up = Vector2.create(d2, Math.toRadians(alpha));
        Vector2 result = start.add(down).add(up);

        double newRotation = (180 + pose.getRotation().getDegrees()) % 360;
        return new Pose2d(result.x, result.y, Rotation2d.fromDegrees(newRotation));
    }

    public Pose2d calcPoseCenter(Pose2d pose, double d) {
        double alpha = pose.getRotation().getDegrees();

        Vector2 start = new Vector2(pose.getX(), pose.getY());
        Vector2 up = Vector2.create(d, Math.toRadians(alpha));
        Vector2 result = start.add(up);

        double newRotation = (180 + pose.getRotation().getDegrees()) % 360;
        return new Pose2d(result.x, result.y, Rotation2d.fromDegrees(newRotation));
    }

    @Override
    public void periodic() {

    }
}
