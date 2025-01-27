package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public LimelightHelpers.PoseEstimate getPoseEstimate(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_Name);
    }

}
