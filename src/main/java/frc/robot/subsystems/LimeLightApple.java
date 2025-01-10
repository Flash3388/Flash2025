/*package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import java.util.Optional;

public class LimeLightApple extends SubsystemBase {

    private static final String LL_NAME = "10.33.88.202";

    private final Swerve swerve;
   // private final AprilTagFieldLayout layout;

    public LimeLightApple(Swerve swerve) {
        this.swerve = swerve;
       // layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        LimelightHelpers.setPipelineIndex(LL_NAME,0);
    }

    public void changePipeLine(double distance) {
        if(distance<=1.5){
            LimelightHelpers.setPipelineIndex(LL_NAME,1);
        } else if (distance>1.5&&distance<=3.5) {
            LimelightHelpers.setPipelineIndex(LL_NAME,0);
        }
    }
    public void recalibratePipeLine(Pose2d robotPose){
        int id = (int)LimelightHelpers.getFiducialID(LL_NAME);
        Optional<Pose3d> aprilTagPoseOptional = layout.getTagPose(id);
        if(aprilTagPoseOptional.isPresent()){
            Pose3d aprilTagPose3d =aprilTagPoseOptional.get();
            Pose2d aprilTagPose2d = aprilTagPose3d.toPose2d();
            double distance = Math.sqrt(
                    Math.pow(aprilTagPose2d.getX()-robotPose.getX(),2)+
                            Math.pow(aprilTagPose2d.getY()-robotPose.getY(),2)
            );
            changePipeLine(distance);
        }
    }

    public boolean targetIsSeen() {
        return LimelightHelpers.getTV(LL_NAME);
    }

    public LimelightHelpers.PoseEstimate getPose2dBlue() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_NAME);
    }

    @Override
    public void periodic(){
        if (targetIsSeen()) {
            LimelightHelpers.PoseEstimate robotPose = getPose2dBlue();
            swerve.updatePoseEstimator(robotPose.pose, robotPose.timestampSeconds);
            recalibratePipeLine(robotPose.pose);
        }

    }
}
*/