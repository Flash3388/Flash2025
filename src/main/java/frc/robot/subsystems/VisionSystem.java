package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.LimelightHelpers;

import java.util.Optional;

public class VisionSystem extends SubsystemBase {

    private static final String LL_NAME_BANANA = "limelight-banana";
    private static final String LL_NANE_APPLE = "limelight-apple";
    private boolean redAlliance;

    private final Limelight limelightBack;
    private final Limelight limelightFront;
    private final AprilTagFieldLayout layout;
    public VisionSystem(){
        limelightBack = new Limelight(LL_NAME_BANANA);
        limelightFront = new Limelight(LL_NANE_APPLE);
        layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        if(DriverStation.getAlliance().isPresent())
            this.redAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public Optional<LimelightHelpers.PoseEstimate> getRobotPoseEstimate(){
        if(limelightFront.hasDetectedTarget()){
            return Optional.of(limelightFront.getPoseEstimate());
        }else if(limelightBack.hasDetectedTarget()){
            return Optional.of(limelightBack.getPoseEstimate());
        }
        return Optional.empty();
    }



}
