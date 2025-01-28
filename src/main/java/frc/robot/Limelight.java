package frc.robot;

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

}
