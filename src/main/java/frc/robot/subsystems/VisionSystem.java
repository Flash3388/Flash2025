package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private final Pose2d coralStationSetPointOffset = new Pose2d(1,0.1,new Rotation2d(0));
    private Pose2d coralReefSetPointLeftOffset;
    private Pose2d coralReefSetPointRightOffset;
    private boolean allianceIsRed;

    private final Limelight limelightBack;
    private final Limelight limelightFront;
    private final AprilTagFieldLayout layout;
    public VisionSystem(){
        limelightBack = new Limelight(LL_NAME_BACK);
        limelightFront = new Limelight(LL_NANE_FRONT);
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        if(DriverStation.getAlliance().isPresent()){
            allianceIsRed = DriverStation.getAlliance().get()== DriverStation.Alliance.Red;
        }

    }

    public Optional<LimelightHelpers.PoseEstimate> getRobotPoseEstimate(){
        if(limelightFront.hasDetectedTarget()){
            return Optional.of(limelightFront.getPoseEstimate());
        }else if(limelightBack.hasDetectedTarget()){
            return Optional.of(limelightBack.getPoseEstimate());
        }
        return Optional.empty();
    }

    public double angleToCoralStation(Pose2d robotPose){
        Pose2d pose1;
        Pose2d pose2;
        List<Pose2d> list = new ArrayList<>();
        if(allianceIsRed){
                pose1 = layout.getTagPose(1).get().toPose2d();
                pose2 = layout.getTagPose(2).get().toPose2d();

        }else{
                pose1 = layout.getTagPose(13).get().toPose2d();
                pose2 = layout.getTagPose(12).get().toPose2d();
        }
        list.add(pose1);
        list.add(pose2);
        Pose2d coralPose = robotPose.nearest(list);
        return coralPose.getRotation().getDegrees();

    }

    public double frontTargetAngle(){
        if(limelightFront.hasDetectedTarget()) {
            int id = limelightFront.getTargetId();
            Pose2d targetPose = layout.getTagPose(id).get().toPose2d();
            return targetPose.getRotation().getDegrees();
        }
        return -1;
    }

    public Optional<Pose2d> pose2dToReef(){
        Optional<Pose2d> optionalPose2d = Optional.empty();
        if(limelightFront.hasDetectedTarget()){
            optionalPose2d = Optional.ofNullable(limelightFront.getTargetPoseRobotSpace());
        }
        return optionalPose2d;
    }

    public Pose2d getCoralStationSetPointOffset(){
        return coralStationSetPointOffset;
    }

    public Pose2d getCoralReefSetPointLeftOffset(){
        return coralReefSetPointLeftOffset;
    }

    public Pose2d getCoralReefSetPointRightOffset(){
        return coralReefSetPointRightOffset;
    }

    public Optional<Pose2d> getIdPose(int id){
        Optional<Pose2d> optionalPose2d = Optional.empty();
        if(layout.getTagPose(id).isPresent())
            optionalPose2d = Optional.ofNullable(layout.getTagPose(id).get().toPose2d());
        return optionalPose2d;
    }

    @Override
    public void periodic(){
        limelightFront.periodic();
        limelightBack.periodic();
    }

}
