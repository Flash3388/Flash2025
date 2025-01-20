package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.LimelightHelpers;

public class LimeLights extends SubsystemBase {

    private static final String LL_Name_Banana = "limelight-banana";
    private static final String LL_Name_Apple = "limelight-apple";
    private boolean redAlliance;

    private final Limelight limelight_banana;
    private final Limelight limelight_apple;
    private final AprilTagFieldLayout layout;
    public LimeLights(){
        limelight_banana = new Limelight(LL_Name_Banana);
        limelight_apple = new Limelight(LL_Name_Apple);
        layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        if(DriverStation.getAlliance().isPresent())
            this.redAlliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public Pose2d getRobotPoseEstimate(){
        if(limelight_apple.targetIsSeen()){
            return limelight_apple.getPoseEstimate().pose;
        }else if(limelight_banana.targetIsSeen()){
            return limelight_banana.getPoseEstimate().pose;
        }
        return  Pose2d.kZero;
    }
    public double getTimeStamp(){

        if(limelight_apple.targetIsSeen()){
            return limelight_apple.getPoseEstimate().timestampSeconds;
        }else if(limelight_banana.targetIsSeen()){
            return limelight_banana.getPoseEstimate().timestampSeconds;
        }
        return 0;
    }

    public boolean targetIsSeenFront(){

        return limelight_apple.targetIsSeen();
    }

    public double distanceFromTargetFront(){
        return limelight_apple.getDistance();
    }

    public boolean targetIsSeenBack(){
        return limelight_banana.targetIsSeen();
    }

    public double distanceFromTargetBack(){
        return limelight_banana.getDistance();
    }

    public double horizontalAngleFront(){
        return limelight_apple.getHorizontalAngle();
    }

    public double verticalAngleFront(){
        return  limelight_apple.getVerticalAngle();
    }

    public double horizontalAngleBack(){
        return limelight_banana.getHorizontalAngle();
    }

    public double verticalAngleBack(){
        return limelight_banana.getVerticalAngle();
    }

    public double distanceToProcessor(Pose2d robotPose){
        Pose2d tagPose;
        if(redAlliance&&layout.getTagPose(3).isPresent()){ //if red
            tagPose = layout.getTagPose(3).get().toPose2d();
        } else if (!redAlliance && layout.getTagPose(16).isPresent()) { //if blue
            tagPose = layout.getTagPose(16).get().toPose2d();
        }else{
            tagPose = Pose2d.kZero;
        }
        return Math.sqrt(
          Math.pow(robotPose.getX()-tagPose.getX(),2) +
          Math.pow(robotPose.getY()-tagPose.getY(),2)
        );
    }

    public double[] getAllFront(){
        LimelightHelpers.RawFiducial[] rawFiducials  = LimelightHelpers.getRawFiducials(LL_Name_Apple);
        double[] arr = new double[rawFiducials.length];
        for(int i=0;i<rawFiducials.length;i++){
            arr[i] = rawFiducials[i].ambiguity;
        }
        return arr;
    }

    public double[] getAllBack(){
        LimelightHelpers.RawFiducial[] rawFiducials  = LimelightHelpers.getRawFiducials(LL_Name_Banana);
        double[] arr = new double[rawFiducials.length];
        for(int i=0;i<rawFiducials.length;i++){
            arr[i] = rawFiducials[i].ambiguity;
        }
        return arr;
    }

    @Override
    public void periodic(){
        limelight_apple.periodic();
        limelight_banana.periodic();
    }
}
