package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSystem;
import swervelib.math.SwerveMath;

public class AllignToFrontTarget extends Command {

    private Swerve swerve;
    private VisionSystem vision;
    private Command command;
    private Pose2d targetPose;

    private double targetAngle;
    private double currentAngle;
    private double finalAngle;
    private double distance;

    public AllignToFrontTarget(Swerve swerveSystem, VisionSystem visionSystem) {
        vision =visionSystem;
        swerve = swerveSystem;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        targetAngle = vision.getMovingAngle();
        targetPose = swerve.getPose();
        currentAngle = targetPose.getRotation().getDegrees();
        distance = vision.getDistance();
        int dir;
        if(currentAngle<targetAngle){
            dir = -1;
        } else {
          dir = 1;
        }
        swerve.getSwerveDrive().drive(SwerveMath.scaleTranslation(new Translation2d(
                       12.3,// targetPose.getX(),
                        4.8), 0.8),
                targetAngle * dir,
                false,
                false);
    }

    @Override
    public void execute() {
        currentAngle = swerve.getPose().getRotation().getDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(targetAngle,currentAngle,5) && MathUtil.isNear(12.3,swerve.getPose().getX(),0.1) && MathUtil.isNear(4.3,swerve.getPose().getY(),0.1);
    }
}
