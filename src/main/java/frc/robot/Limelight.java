package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.Swerve;

public class Limelight {
    private final String LL_Name;


    public Limelight(String LL_Name){
        this.LL_Name = LL_Name;
        LimelightHelpers.setPipelineIndex(LL_Name,1);
    }

    public boolean targetIsSeen(){
        return LimelightHelpers.getTV(LL_Name);
    }

    public double getDistance(){
        return LimelightHelpers.getRawFiducials(LL_Name)[5].distToRobot;
    }

    public double getHorizontalAngle(){
        return LimelightHelpers.getTX(LL_Name);
    }

    public double getVerticalAngle(){
        return LimelightHelpers.getTX(LL_Name);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimate(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_Name);
    }

    public LimelightHelpers.RawFiducial[] getAll(){
        return LimelightHelpers.getRawFiducials(LL_Name);
    }

    public void changePipeLine(double distance){
        if(distance >1.5 && distance <3.5){
            LimelightHelpers.setPipelineIndex(LL_Name,1);
        }else if(distance<1.5){
            LimelightHelpers.setPipelineIndex(LL_Name,0);
        }
    }

    public void periodic(){

    }
}
