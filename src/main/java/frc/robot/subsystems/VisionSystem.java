package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.LimelightHelpers;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionSystem extends SubsystemBase {

    private static final String LL_NAME_BACK = "limelight-back";
    private static final String LL_NANE_FRONT = "limelight-front";
    private final List<Pose2d> reefPositions;
    private boolean allianceIsRed;
    private Translation2d translation2d;

    private final Limelight limelightBack;
    private final Limelight limelightFront;
    private final AprilTagFieldLayout layout;

    public VisionSystem() {
        limelightBack = new Limelight(LL_NAME_BACK);
        limelightFront = new Limelight(LL_NANE_FRONT);
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        if (DriverStation.getAlliance().isPresent()) {
            allianceIsRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        }
        reefPositions = new ArrayList<>();
        setReefPositions();

    }

    public Optional<LimelightHelpers.PoseEstimate> getRobotPoseEstimate(){
        if(limelightFront.hasDetectedTarget()) {
            if (limelightFront.getDistanceToTarget() <= 2.5)
                return Optional.of(limelightFront.getPoseEstimate());
        } else if(limelightBack.hasDetectedTarget()) {
            if (limelightBack.getDistanceToTarget() <= 2.5)
               return Optional.of(limelightBack.getPoseEstimate());
        }
        return Optional.empty();
    }


    public Optional<Double> frontTargetAngle(){
        Optional<Double> angle = Optional.empty();
        if(limelightFront.hasDetectedTarget()) {
            int id = limelightFront.getTargetId();
            Pose2d targetPose = layout.getTagPose(id).get().toPose2d();
            angle = Optional.of(targetPose.getRotation().getDegrees()) ;
        }
        return angle;
    }

    public void changePipeline(int id){
        limelightFront.changePipeline(id);
        limelightBack.changePipeline(id);
    }

    private void setReefPositions() {
        Pose2d pose;
        if (allianceIsRed) {
            for (int i = 7; i < 12; i++) {
                pose = layout.getTagPose(i).get().toPose2d();
                reefPositions.add(pose);
            }
        } else {
            for (int i = 17; i < 23; i++) {
                pose = layout.getTagPose(i).get().toPose2d();
                reefPositions.add(pose);
            }
        }
    }

    private Pose2d getClosestReefPose(Pose2d robotPose) {
        Pose2d closestPose = reefPositions.get(0);
        double closestDistance = Math.sqrt(
                Math.pow(closestPose.getX() - robotPose.getX(), 2) +
                        Math.pow(closestPose.getY() - robotPose.getY(), 2));
        for (int i = 1; i < 6; i++) {
            Pose2d pose = reefPositions.get(i);
            double distance = Math.sqrt(
                    Math.pow(pose.getX() - robotPose.getX(), 2) +
                            Math.pow(pose.getY() - robotPose.getY(), 2));
            if (closestDistance > distance) {
                closestPose = pose;
                closestDistance = distance;
            }
        }
        return closestPose;
    }
    public double getMovingAngleReef(int id){
        switch (id){
            case 3: return 90;
            case 9: return -120;
            case 8: return -90;
        }
        return 0;
    }

    public double getMovingAngleCoral(int id){
        switch (id){
            case 2: return 130;
        }
        return 0;
    }

    public Pose2d getMovingPoseCoral(int id){
        switch (id){
            case 2: return new Pose2d(16.4,7.14,new Rotation2d(130));
        }
        return Pose2d.kZero;
    }

    public Pose2d getMovingPoseReefLeft(int id){
        switch (id){
            case 8: return new Pose2d(13.73,4.75,new Rotation2d(-120));
        }
        return getRobotPoseEstimate().get().pose;
    }

    public double getDistance(){
        return limelightFront.getFiducials()[0].distToRobot;
    }

    public Pose2d getReefPoseLeft(Pose2d robotPose) {
        Pose2d pose = getClosestReefPose(robotPose);
        Pose2d leftPose;
        pose.getRotation().rotateBy(Rotation2d.k180deg);
        leftPose = pose.plus(new Transform2d(1, 2, Rotation2d.kZero));
        return leftPose;
    }

    public Pose2d getReefPoseRight(Pose2d robotPose) {
        Pose2d pose = getClosestReefPose(robotPose);
        Pose2d rightPose;
        pose.getRotation().rotateBy(Rotation2d.k180deg);
        rightPose = pose.plus(new Transform2d(1, 2, Rotation2d.kZero));
        return rightPose;
    }

    public double angleToCoralStation(Pose2d robotPose) {
        Pose2d pose1;
        Pose2d pose2;
        List<Pose2d> list = new ArrayList<>();
        if (allianceIsRed) {
            pose1 = layout.getTagPose(1).get().toPose2d();
            pose2 = layout.getTagPose(2).get().toPose2d();

        } else {
            pose1 = layout.getTagPose(13).get().toPose2d();
            pose2 = layout.getTagPose(12).get().toPose2d();
        }
        list.add(pose1);
        list.add(pose2);
        Pose2d coralPose = robotPose.nearest(list);
        return coralPose.getRotation().getDegrees();

    }

    public double getFrontAngle() {
        return limelightFront.getAngle();
    }

    public Optional<Pose2d> pose2dToReef() {
        Optional<Pose2d> optionalPose2d = Optional.empty();
        if (limelightFront.hasDetectedTarget()) {
            optionalPose2d = Optional.ofNullable(limelightFront.getTargetPoseRobotSpace());
        }
        return optionalPose2d;
    }

    public Optional<Pose2d> getIdPose(int id) {
        Optional<Pose2d> optionalPose2d = Optional.empty();
        if (layout.getTagPose(id).isPresent())
            optionalPose2d = Optional.ofNullable(layout.getTagPose(id).get().toPose2d());
        return optionalPose2d;
    }

    public int frontGetTargetId() {
        return limelightFront.getTargetId();
    }
    public int backGetTargetId(){return limelightBack.getTargetId();}


    public double getDistanceFront(){
        return limelightFront.getDistance();
    }

    public boolean frontHasSeenTarget() {
        return limelightFront.hasDetectedTarget();
    }

    @Override
    public void periodic() {
        // limelightFront.periodic();
        // limelightBack.periodic();
    }
}
