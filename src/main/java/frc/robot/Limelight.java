package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class Limelight {
    private final String name;


    public Limelight(String LL_Name){
        this.name = LL_Name;
        LimelightHelpers.setPipelineIndex(LL_Name,1);
    }

    public boolean hasDetectedTarget(){
        return LimelightHelpers.getTV(name);
    }

    public double getDistanceToTarget(){
        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
        if(fiducials.length >0)
            return fiducials[0].distToRobot;
        return 3;
    }

    public int getTargetId(){
        return  (int) LimelightHelpers.getFiducialID(name);
    }

    public double getTargetArea(){
        return LimelightHelpers.getTA(name);
    }

    public void changePipeline(int id){
            LimelightHelpers.setPipelineIndex(name,id);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimate(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

}
