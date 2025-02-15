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
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public Optional<LimelightHelpers.PoseEstimate> getRobotPoseEstimate() {
        double distanceToTargetFront = limelightFront.hasDetectedTarget() ?
                limelightFront.getDistanceToTarget() :
                Double.MAX_VALUE;
        double distanceToTargetBack = limelightBack.hasDetectedTarget() ?
                limelightBack.getDistanceToTarget() :
                Double.MAX_VALUE;

        // always use the closest measure
        if (distanceToTargetFront < distanceToTargetBack) {
            // use front
            if (distanceToTargetFront <= RobotMap.LIMELIGHT_DISTANCE_TO_TARGET_LIMIT) {
                return Optional.of(limelightFront.getPoseEstimate());
            }
        } else {
            // use back
            if (distanceToTargetBack <= RobotMap.LIMELIGHT_DISTANCE_TO_TARGET_LIMIT) {
                return Optional.of(limelightBack.getPoseEstimate());
            }
        }

        return Optional.empty();
    }

    public Pose2d getAprilTagPose(int id) {
        return layout.getTagPose(id).orElseThrow().toPose2d();
    }

    public Pose2d getPoseForReefStand(int id, ReefStandRow row) {
        Pose2d pose = getAprilTagPose(id);

        Pose2d calculatedPose = calcPoseTwoSides(pose, RobotMap.OFFSET_ON_STAND_REEF, RobotMap.OFFSET_REEF, row == ReefStandRow.LEFT);
        return new Pose2d(calculatedPose.getX(), calculatedPose.getY(), calculatedPose.getRotation());
    }

    public Pose2d getPoseForFeeder(int id, FeederSide side) {
        Pose2d pose = getAprilTagPose(id);

        Pose2d calculatedPose;
        switch (side) {
            case LEFT:
                calculatedPose = calcPoseTwoSides(pose, RobotMap.OFFSET_ON_STAND_FEEDER, RobotMap.OFFSET_FEEDER, true);
                break;
            case RIGHT:
                calculatedPose = calcPoseTwoSides(pose, RobotMap.OFFSET_ON_STAND_FEEDER, RobotMap.OFFSET_FEEDER, false);
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
        Pose2d pose = getAprilTagPose(id);

        Pose2d calculatedPose = calcPoseCenter(pose, RobotMap.OFFSET_PROCESSOR);
        return new Pose2d(calculatedPose.getX(), calculatedPose.getY(), calculatedPose.getRotation());
    }

    private Pose2d calcPoseTwoSides(Pose2d pose, double d1, double d2, boolean isLeft) {
        double alpha = pose.getRotation().getDegrees();
        double beta = isLeft ? alpha - 90 : alpha + 90;

        Vector2 start = new Vector2(pose.getX(), pose.getY());
        Vector2 down = Vector2.create(d1, Math.toRadians(beta));
        Vector2 up = Vector2.create(d2, Math.toRadians(alpha));
        Vector2 result = start.add(down).add(up);

        double newRotation = (180 + pose.getRotation().getDegrees()) % 360;
        return new Pose2d(result.x, result.y, Rotation2d.fromDegrees(newRotation));
    }

    private Pose2d calcPoseCenter(Pose2d pose, double d) {
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
