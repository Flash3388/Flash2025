package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.LimelightHelpers;

public class Limelight {
    private final String LL_Name;
    private final boolean isFront;

    private final Swerve swerve;
    private final AprilTagFieldLayout layout;

    public Limelight(Swerve swerveSystem,String LL_Name, boolean isFront){
        this.swerve = swerveSystem;
        this.LL_Name = LL_Name;
        this.isFront = isFront;
        layout = new AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        LimelightHelpers.setPipelineIndex(LL_Name,0);
    }

    public boolean targetIsSeen(){
        Li
    }
}
