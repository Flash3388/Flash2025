package frc.robot;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeGripper;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralGripper;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSystem;

import java.util.*;

public class Robot extends TimedRobot {

    private Swerve swerve;
    private VisionSystem visionSystem;
    private AlgaeArm algaeArm;
    private AlgaeGripper algaeGripper;
    private CoralElevator coralElevator;
    private CoralGripper coralGripper;
    private CoralArm coralArm;

    private PneumaticHub pneumaticsHub;
    private Compressor compressor;
    private Dashboard dashboard;

    private CoralArmCommand coralArmCommand;
    private Command autoCommand;

    private XboxController xbox;
    private SendableChooser<Command> autoChooser;

    @Override
    public void robotInit() {
        swerve = new Swerve();
        visionSystem = new VisionSystem();
        algaeArm = new AlgaeArm();
        algaeGripper = new AlgaeGripper();
        coralElevator = new CoralElevator();
        coralGripper = new CoralGripper();
        coralArm = new CoralArm();

        xbox = new XboxController(0);

        pneumaticsHub = new PneumaticHub();
        compressor = new Compressor(RobotMap.COMPRESSION_PORT, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(RobotMap.MIN_PRESSURE, RobotMap.MAX_PRESSURE);

        dashboard = new Dashboard(algaeArm, algaeGripper, coralElevator, coralArm, coralGripper);

        coralArmCommand = new CoralArmCommand(coralArm);
        coralArm.setDefaultCommand(coralArmCommand);

        swerve.setDefaultCommand(swerve.driveA(
                () -> MathUtil.applyDeadband(-xbox.getLeftY(), 0.05),
                () -> MathUtil.applyDeadband(-xbox.getLeftX(), 0.05),
                () -> MathUtil.applyDeadband(-xbox.getRightX(), 0.05)
        ));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

       /* Command checkIfAlgaeRetract = Commands.defer(()->{
            if(algaeArm.isExtended()){
                return new RetractAlgaeArm(algaeArm);
            }
            return Commands.idle(algaeArm);
        }, Set.of(algaeArm));
        algaeArm.setDefaultCommand(checkIfAlgaeRetract); */

       /* Command checkIfLow = Commands.defer(()-> {
            if(coralElevator.isRaised()){
                return new LowerCoralElevator(coralElevator);
            }
            return Commands.idle(coralElevator);
        }, Set.of(coralElevator));
        coralElevator.setDefaultCommand(checkIfLow);*/

       /* Command checkCoral = Commands.defer(()-> {
            if (coralGripper.hasCoral()) {
                return new HoldCoral(coralGripper);
            }
            return Commands.idle(coralGripper);
        }, Set.of(coralGripper));
        coralGripper.setDefaultCommand(checkCoral);*/

        /*Command checkAlgae = Commands.defer(()-> {
            if (algaeGripper.hasAlgae()) {
                return new HoldAlgae(algaeGripper);
            }
            return Commands.idle(algaeGripper);
        }, Set.of(algaeGripper));

        algaeGripper.setDefaultCommand(checkAlgae);*/

        new JoystickButton(xbox, XboxController.Button.kY.value)
                .onTrue(driveToCoralReefDownAndPlaceCoral(8, ReefStandRow.RIGHT, false));
        new JoystickButton(xbox, XboxController.Button.kA.value)
                .onTrue(driveToReef8Special(ReefStandRow.RIGHT));
        new JoystickButton(xbox, XboxController.Button.kX.value)
                .onTrue(new ReleaseAlgae(algaeGripper));
        new JoystickButton(xbox, XboxController.Button.kB.value)
                .onTrue(driveToNearestReef(ReefStandRow.RIGHT));
        new JoystickButton(xbox, XboxController.Button.kRightBumper.value)
                .onTrue(twoCoralsAutoLeftRightSpecialReef8Feeder2());
        new JoystickButton(xbox, XboxController.Button.kLeftBumper.value)
                .onTrue(coralCollect());
        new JoystickButton(xbox,XboxController.Button.kLeftStick.value)
                .onTrue(new LowerCoralElevator(coralElevator));

        new POVButton(xbox, 270).onTrue(driveToCoralReefUp(8, ReefStandRow.RIGHT));
        new POVButton(xbox, 90).onTrue(driveAndCollectFromFeeder(2, FeederSide.RIGHT));
        new POVButton(xbox, 180).onTrue(driveToFeeder(2, FeederSide.CENTER));
        new POVButton(xbox, 0).onTrue(level3AndAlgaeAndCollectAndPlace(8, 2, 3));
    }


    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Pressure", pneumaticsHub.getPressure(0));

        Optional<LimelightHelpers.PoseEstimate> pose = visionSystem.getRobotPoseEstimate();
        if (pose.isPresent()) {
            LimelightHelpers.PoseEstimate poseEstimate = pose.get();
            swerve.updatePoseEstimator(poseEstimate);

            if (poseEstimate.rawFiducials.length > 0) {
                SmartDashboard.putNumber("AprilTagID", poseEstimate.rawFiducials[0].id);
            }
        } else {
            SmartDashboard.putNumber("AprilTagID", -1);
        }

        OptionalInt nearestAprilTagOpt = findNearestAprilTagForCurrentPose();
        if (nearestAprilTagOpt.isPresent()) {
            int aprilTagId = nearestAprilTagOpt.getAsInt();
            Pose2d aprilTag = visionSystem.getAprilTagPose(aprilTagId);

            swerve.getField().getObject("NearestAprilTag").setPose(aprilTag);
            SmartDashboard.putNumber("NearestAprilTagID", aprilTagId);
        } else {
            swerve.getField().getObject("NearestAprilTag").setPoses();
            SmartDashboard.putNumber("NearestAprilTagID", -1);
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

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {
        this.autoCommand = autoChooser.getSelected();
        if (this.autoCommand != null) {
            this.autoCommand.schedule();
        }

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {
        if (this.autoCommand != null) {
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

    private Command twoCoralsAutoLeftRightSpecialReef8Feeder2() {
        return new SequentialCommandGroup(
                driveToCoralReef8SpecialDownAndPlaceCoral(ReefStandRow.RIGHT, true),
                Commands.runOnce(()-> DriverStation.reportWarning("Go To Feeder", false)),
                driveAndCollectFromFeeder(2, FeederSide.CENTER),
                Commands.runOnce(()-> DriverStation.reportWarning("Bitch Please", false)),
                driveToCoralReefDownAndPlaceCoral(8, ReefStandRow.LEFT, true)
        );
    }

    private Command driveToCoralReef8SpecialDownAndPlaceCoral(ReefStandRow row, boolean level3) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                driveToCoralReefDownAndPlaceCoral(8, row, level3)
                //level3 ? coralLevel3Place(false) : coralLevel2Place(false)
        );
    }

    private Command level3AndAlgaeAndCollectAndPlace(int reefAprilTagId, int feedAprilTagId, int processorAprilTagId) {
        return new SequentialCommandGroup(
                driveToCoralReefUp(reefAprilTagId, ReefStandRow.RIGHT),
                driveToProcessorAndPlaceAlgae(processorAprilTagId),
                driveAndCollectFromFeeder(feedAprilTagId, FeederSide.CENTER),
                driveToCoralReefDownAndPlaceCoral(reefAprilTagId, ReefStandRow.LEFT, true)
        );
    }

    private Command driveToCoralReefUp(int aprilTagId, ReefStandRow row) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        driveToReef(aprilTagId, row),
                        new SequentialCommandGroup(
                                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                                new ExtendedAlgaeArm(algaeArm),
                                new RaiseCoralElevator(coralElevator)
                        )
                ),
                new ParallelCommandGroup(
                        coralLevel3Place(false),
                        new CollectAlgae(algaeGripper)
                ),
                new LowerCoralElevator(coralElevator)
        );
    }

    private Command driveAndCollectFromFeeder(int aprilTagId, FeederSide side) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        driveToFeeder(aprilTagId, side),
                        new SequentialCommandGroup(
                                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                                new LowerCoralElevator(coralElevator),
                                new CollectCoral(coralGripper)
                        )
                )
        );
    }

    private Command driveToCoralReefDownAndPlaceCoral(int aprilTagId, ReefStandRow row, boolean level3) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                driveToReef(aprilTagId, row),
                level3 ? coralLevel3Place(false) : coralLevel2Place(false)
                ),
                new ParallelCommandGroup(
                new ReleaseCoral(coralGripper),
                        algaeCollect())
        );
    }

    private Command driveToProcessorAndPlaceAlgae(int aprilTagId) {
        return new SequentialCommandGroup(
                driveToProcessor(aprilTagId),
                algaeOut()
        );
    }

    private Command coralLevel2Place(boolean alsoMoveArm) {
        return new SequentialCommandGroup(
                alsoMoveArm ?
                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)) :
                        Commands.none(),
                new ParallelCommandGroup(
                        new LowerCoralElevator(coralElevator),
                        Commands.waitUntil(() -> coralArmCommand.didReachTargetPosition())
                )
        );
    }

    private Command coralLevel3Place(boolean alsoMoveArm) {
        return new SequentialCommandGroup(
                alsoMoveArm ?
                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)) :
                        Commands.none(),
                new ParallelCommandGroup(
                        new RaiseCoralElevator(coralElevator),
                        Commands.waitUntil(() -> coralArmCommand.didReachTargetPosition())
                )
        );
    }

    private Command coralCollect() {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                new ParallelCommandGroup(
                        new LowerCoralElevator(coralElevator),
                        Commands.waitUntil(() -> coralArmCommand.didReachTargetPosition())
                ),
                new CollectCoral(coralGripper)
        );
    }

    private Command algaeCollect() {
        return new SequentialCommandGroup(
                new ExtendedAlgaeArm(algaeArm),
                new CollectAlgae(algaeGripper)
        );
    }

    private Command algaeOut() {
        return new SequentialCommandGroup(
                new LowerCoralElevator(coralElevator),
                new ReleaseAlgae(algaeGripper),
                new RetractAlgaeArm(algaeArm)
        );
    }

    private Command driveToReef8Special(ReefStandRow row) {
        Pose2d pose = visionSystem.getPoseForReefStand(8, row);
        Pose2d middlePose = new Pose2d(13.189, 6.146, Rotation2d.fromDegrees(-90));
        return driveToPoseStraight(middlePose, pose);
    }

    private Command driveToNearestReef(ReefStandRow row) {
        return Commands.defer(()-> {
            OptionalInt aprilTagIdOptional = findNearestAprilTagForCurrentPose();
            if (aprilTagIdOptional.isEmpty()) {
                return Commands.none();
            }

            return driveToReef(aprilTagIdOptional.getAsInt(), row);
        }, Set.of(swerve));
    }

    private Command driveToReef(int aprilTagId, ReefStandRow row) {
        Pose2d pose = visionSystem.getPoseForReefStand(aprilTagId, row);
        return driveToPose(pose);
    }

    private Command driveToProcessor(int aprilTagId) {
        Pose2d pose = visionSystem.getPoseToProcessor(aprilTagId);
        return driveToPose(pose);
    }

    private Command driveToFeeder(int aprilTagId, FeederSide side) {
        Pose2d pose = visionSystem.getPoseForFeeder(aprilTagId, side);
        // rotate 180 because we want our back to the feeder
        Pose2d rotated = new Pose2d(pose.getX(), pose.getY(), pose.getRotation().rotateBy(Rotation2d.k180deg));
        return driveToPose(rotated);
    }

    private Command driveToPose(Pose2d pose) {
        return new SequentialCommandGroup(
                Commands.runOnce(()-> swerve.getField().getObject("Target").setPose(pose)),
                AutoBuilder.pathfindToPose(pose, RobotMap.CONSTRAINTS),
                Commands.runOnce(()-> swerve.getField().getObject("Target").setPoses())
        );
    }

    private Command driveToPoseStraight(Pose2d... poses) {
        return Commands.defer(()-> {
            List<Pose2d> posesList = new ArrayList<>();
            posesList.add(swerve.getPose());
            posesList.addAll(Arrays.asList(poses));

            List<PathPoint> pathPoints = posesList.stream()
                    .map((pose)-> new PathPoint(
                            pose.getTranslation(),
                            null,
                            RobotMap.CONSTRAINTS
                    ))
                    .toList();

            PathPlannerPath path = PathPlannerPath.fromPathPoints(
                    pathPoints,
                    RobotMap.CONSTRAINTS,
                    new GoalEndState(0, poses[poses.length - 1].getRotation()));
            path.preventFlipping = true;

            return new SequentialCommandGroup(
                    Commands.runOnce(()-> swerve.getField().getObject("Target").setPoses(poses)),
                    AutoBuilder.followPath(path),
                    Commands.runOnce(()-> swerve.getField().getObject("Target").setPoses())
            );
        }, Set.of(swerve));
    }

    private OptionalInt findNearestAprilTagForCurrentPose() {
        Pose2d pose = swerve.getPose();

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        int sideIndex = alliance == DriverStation.Alliance.Red ? 0 : 1;

        return findNearestAprilTag(pose, RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE[sideIndex]);
    }

    private OptionalInt findNearestAprilTag(Pose2d pose, int[] aprilTagIds) {
        int foundTag = -1;
        double shortestDistance = Double.MAX_VALUE;

        for (int id : aprilTagIds) {
            Pose2d aprilTagPose = visionSystem.getAprilTagPose(id);
            double distance = pose.getTranslation().getDistance(aprilTagPose.getTranslation());

            if (distance < shortestDistance) {
                foundTag = id;
                shortestDistance = distance;
            }
        }

        if (foundTag < 0) {
            return OptionalInt.empty();
        }

        return OptionalInt.of(foundTag);
    }
}
