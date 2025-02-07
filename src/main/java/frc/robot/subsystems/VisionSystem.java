package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.LimelightHelpers;

import java.util.Optional;

public class VisionSystem extends SubsystemBase {

    private static final String LL_NAME_BACK = "limelight-back";
    private static final String LL_NANE_FRONT = "limelight-front";
    private boolean redAlliance;

    private final Limelight limelightBack;
    private final Limelight limelightFront;
    private final AprilTagFieldLayout layout;
    public VisionSystem(){
        limelightBack = new Limelight(LL_NAME_BACK);
        limelightFront = new Limelight(LL_NANE_FRONT);
        layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    public Optional<LimelightHelpers.PoseEstimate> getRobotPoseEstimate(){
        if(limelightFront.hasDetectedTarget()){
            if(limelightFront.getDistanceToTarget()<=2.5)
                return Optional.of(limelightFront.getPoseEstimate());
        }else if(limelightBack.hasDetectedTarget()){
            if(limelightBack.getDistanceToTarget()<=2.5)
             return Optional.of(limelightBack.getPoseEstimate());
        }
        return Optional.empty();
    }

    public void changePipeline(int id){
        limelightFront.changePipeline(id);
        limelightBack.changePipeline(id);
    }



}
