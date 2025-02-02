package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSystem;

import java.util.Optional;

public class Robot extends TimedRobot {
    private Command autoCommand;
    private Swerve swerve;
    private XboxController xbox;
    private SendableChooser<Command> autoChooser;
    private VisionSystem visionSystem;
    @Override
    public void robotInit() {
        swerve = new Swerve();
        xbox = new XboxController(0);
        visionSystem = new VisionSystem();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    @Override
    public void robotPeriodic() {
        Optional<LimelightHelpers.PoseEstimate> pose = visionSystem.getRobotPoseEstimate();
        if(pose.isPresent()) {
            swerve.updatePoseEstimator(pose.get());
        }
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void teleopInit() {
        swerve.driveA(
                ()-> MathUtil.applyDeadband(-xbox.getLeftY(), 0.05),
                ()-> MathUtil.applyDeadband(-xbox.getLeftX(), 0.05),
                ()-> MathUtil.applyDeadband(-xbox.getRightX(), 0.05)
        ).schedule();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {
        this.autoCommand = autoChooser.getSelected();;
        if(this.autoCommand  != null) {
            this.autoCommand.schedule();
        }

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {
     if (this.autoCommand != null){
            this.autoCommand.cancel();
            this.autoCommand = null;
      }
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }
}

