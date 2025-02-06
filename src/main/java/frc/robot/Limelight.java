package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.Optional;

public class Limelight {
    private final String name;


    public Limelight(String LL_Name){
        this.name = LL_Name;
        LimelightHelpers.setPipelineIndex(LL_Name,1);
    }

    public boolean hasDetectedTarget(){
        return LimelightHelpers.getTV(name);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimate(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    public int aprilTagId(){
        if(hasDetectedTarget()){
            return(int)LimelightHelpers.getFiducialID(name);
        }
        return -1;
    }

    public Pose2d getTargetPoseRobotSpace(){
        return LimelightHelpers.getTargetPose3d_RobotSpace(name).toPose2d();
    }

    public void changePipeLine(){
        if(hasDetectedTarget()){
                double distance = LimelightHelpers.getRawFiducials(name)[0].distToRobot;
            if(distance<=1.5)
                LimelightHelpers.setPipelineIndex(name,0);
        } else
            LimelightHelpers.setPipelineIndex(name,1);
    }

    public int getTargetId(){
        return  (int) LimelightHelpers.getFiducialID(name);
    }

    public void periodic(){
        changePipeLine();
    }



}
