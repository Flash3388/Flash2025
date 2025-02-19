package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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
import frc.robot.subsystems.*;

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
    private LedLights leds;

    private CoralArmCommand coralArmCommand;

    private XboxController xboxSecond;
    private XboxController xboxMain;

    private boolean isAuto = true;
    private SendableChooser<String> feederAuto;

    @Override
    public void robotInit() {
        swerve = new Swerve();
        visionSystem = new VisionSystem();
        algaeArm = new AlgaeArm();
        algaeGripper = new AlgaeGripper();
        coralElevator = new CoralElevator();
        coralGripper = new CoralGripper();
        coralArm = new CoralArm();

        xboxSecond = new XboxController(0);
        xboxMain = new XboxController(1);

        pneumaticsHub = new PneumaticHub();
        compressor = new Compressor(RobotMap.COMPRESSION_PORT, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(RobotMap.MIN_PRESSURE, RobotMap.MAX_PRESSURE);

        dashboard = new Dashboard(algaeArm, algaeGripper, coralElevator, coralArm, coralGripper);
        leds = new LedLights();

        coralArmCommand = new CoralArmCommand(coralArm);
        coralArm.setDefaultCommand(coralArmCommand);

        swerve.setDefaultCommand(swerve.driveA(
                () -> MathUtil.applyDeadband(-xboxMain.getLeftY(), 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getLeftX(), 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getRightX(), 0.05)
        ));
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
        configureButtons();
        SmartDashboard.putBoolean("isAuto",true);

        feederAuto = new SendableChooser<String>();
        feederAuto.setDefaultOption("center","CENTER");
        feederAuto.addOption("left","LEFT");
        feederAuto.addOption("right","RIGHT");
        SmartDashboard.putData("feederAutomation",feederAuto);
        SmartDashboard.putBoolean("isLeft",true);
    }

    public void configureButtons(){
        new JoystickButton(xboxSecond, XboxController.Button.kY.value)
                .onTrue(isAuto ? driveAndCollectFromFeeder(FeederSide.CENTER) : new RaiseCoralElevator(coralElevator));
        new JoystickButton(xboxSecond, XboxController.Button.kA.value)
                .onTrue(new LowerCoralElevator(coralElevator));
        new JoystickButton(xboxSecond, XboxController.Button.kX.value)
                .onTrue(isAuto ? driveAndCollectFromFeeder(FeederSide.LEFT) :coralCollect());
        new JoystickButton(xboxSecond, XboxController.Button.kB.value)
                .onTrue(isAuto ? driveAndCollectFromFeeder(FeederSide.RIGHT) :coralPut());
        new JoystickButton(xboxSecond, XboxController.Button.kRightBumper.value)
                .onTrue(isAuto ? driveToProcessorAndPlaceAlgae() :new RemoveSwerveCommand(swerve));
        new JoystickButton(xboxSecond, XboxController.Button.kLeftBumper.value)
                .onTrue(Commands.none());
        if(isAuto){
        new POVButton(xboxSecond, 315).onTrue(driveToNearestReefAndPut(ReefStandRow.LEFT,true));
        new POVButton(xboxSecond, 135).onTrue(driveToNearestReefAndPut(ReefStandRow.RIGHT,false));
        new POVButton(xboxSecond, 45).onTrue(driveToNearestReefAndPut(ReefStandRow.RIGHT,true));
        new POVButton(xboxSecond, 225).onTrue(driveToNearestReefAndPut(ReefStandRow.LEFT,false));
        }else {
            new POVButton(xboxSecond,0).onTrue(new ExtendedAlgaeArm(algaeArm));
            new POVButton(xboxSecond,180).onTrue(new RetractAlgaeArm(algaeArm));
            new POVButton(xboxSecond,270).onTrue(new CollectAlgae(algaeGripper));
            new POVButton(xboxSecond,90).onTrue(new ReleaseAlgae(algaeGripper));
        }
    }


    @Override
    public void robotPeriodic() {
        LEDPattern newPattern;

        if (coralGripper.hasCoral() && algaeGripper.hasAlgae()) {
            SmartDashboard.putString("ledsMode", "Algae + Coral");
            newPattern = LEDPattern.solid(Color.kTurquoise);
        } else if (algaeGripper.hasAlgae()) {
            SmartDashboard.putString("ledsMode", "Algae");
            newPattern = LEDPattern.solid(Color.kPurple);
        } else if (coralGripper.hasCoral()) {
            SmartDashboard.putString("ledsMode", "Coral");
            newPattern = LEDPattern.solid(Color.kWhite);
        } else {
            SmartDashboard.putString("ledsMode", "Default");
            newPattern = leds.getFlashPattern();
        }

        // Only update if the pattern has changed
        if (!newPattern.equals(leds.getCurrentPattern())) {
            leds.setPattern(newPattern);
        }

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

        OptionalInt nearestAprilTagOpt = findNearestAprilTagForCurrentPose(0,1);
        if (nearestAprilTagOpt.isPresent()) {
            int aprilTagId = nearestAprilTagOpt.getAsInt();
            Pose2d aprilTag = visionSystem.getAprilTagPose(aprilTagId);

            swerve.getField().getObject("NearestAprilTag").setPose(aprilTag);
            SmartDashboard.putNumber("NearestAprilTagID", aprilTagId);
        } else {
            swerve.getField().getObject("NearestAprilTag").setPoses();
            SmartDashboard.putNumber("NearestAprilTagID", 8);
        }


        OptionalInt nearestAprilTagFeederOpt = findNearestAprilTagForCurrentPose(2,3);
        if (nearestAprilTagFeederOpt.isPresent()) {
            int aprilTagId = nearestAprilTagFeederOpt.getAsInt();
            Pose2d aprilTag = visionSystem.getAprilTagPose(aprilTagId);

            swerve.getField().getObject("NearestAprilTagFeeder").setPose(aprilTag);
            SmartDashboard.putNumber("NearestAprilTagFeederID", aprilTagId);
        } else {
            swerve.getField().getObject("NearestAprilTagFeeder").setPoses();
            SmartDashboard.putNumber("NearestAprilTagFeederID", 2);
        }

        boolean isAuto1 = SmartDashboard.getBoolean("isAuto",true);
        if(isAuto1 != isAuto){
            isAuto = isAuto1;
            configureButtons();
        }

        Optional<LimelightHelpers.PoseEstimate> optionalPoseEstimate = visionSystem.getRobotPoseEstimate();
        SmartDashboard.putBoolean("Pose Exists",optionalPoseEstimate.isPresent());
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
        RobotMap.LIMELIGHT_DISTANCE_TO_TARGET_LIMIT = 4;
        visionSystem.changePipeLine(0);
    }

    @Override
    public void disabledPeriodic() {
        Optional<LimelightHelpers.PoseEstimate> poseEstimate= visionSystem.getRobotPoseEstimate();
        if(poseEstimate.isPresent()){
            LimelightHelpers.PoseEstimate pose = poseEstimate.get();
            swerve.updatePoseEstimator(pose);
        }
    }

    @Override
    public void disabledExit() {
        RobotMap.LIMELIGHT_DISTANCE_TO_TARGET_LIMIT = 2.7;
        visionSystem.changePipeLine(1);
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

    Command autoCommand;
    @Override
    public void autonomousInit() {
        boolean isLeft = SmartDashboard.getBoolean("isLeft",true);
        autoCommand = orbitAuto();
        autoCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {
        autoCommand.cancel();
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

    private Command orbitAuto(){
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0:1;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        reefAuto(ReefStandRow.RIGHT,aprilTags[sideIndexReef][5],true),
                        algaeCollect()),
                ProcessorAuto()
        );
    }

    private Command threeCorals(boolean isLeft){
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0:1;
        int sideIndexFeeder = isRed() ? 2:3;
        int indexReef = isLeft ? 1:3;
        int indexFeeder = isLeft? 1 : 0;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                reefAuto(ReefStandRow.RIGHT,aprilTags[sideIndexReef][indexReef],true),
                algaeCollect()),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()),aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.LEFT,aprilTags[sideIndexReef][indexReef],true),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()),aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.LEFT,aprilTags[sideIndexReef][indexReef],false)
        );
    }

    private Command algaeAuto(boolean isLeft){
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0:1;
        int sideIndexFeeder = isRed() ? 2:3;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                reefAuto(ReefStandRow.RIGHT,aprilTags[sideIndexReef][1],true),
                algaeCollect()),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()),aprilTags[sideIndexFeeder][1]),
                Commands.none(),
                reefAuto(ReefStandRow.LEFT,aprilTags[sideIndexReef][1],true),
                ProcessorAuto()
        );
    }

    private Command ProcessorAuto(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                driveToProcessor(),
                new LowerCoralElevator(coralElevator)),
                algaeOut()
        );
    }

    private Command feederAuto(FeederSide side,int aprilTagId){
        return new ParallelCommandGroup(
            driveToFeeder(aprilTagId,side),
            coralCollect()
        );
    }

    private Command reefAuto(ReefStandRow row, int aprilTagId, boolean level3) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                driveToReef(aprilTagId,row),
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                new SequentialCommandGroup(
                new DistanceDelay(visionSystem,swerve,aprilTagId),
                level3 ? new RaiseCoralElevator(coralElevator) : Commands.none())),
                new ReleaseCoral(coralGripper)
        );
    }

    private Command driveAndCollectFromFeeder(FeederSide side) {
       /* OptionalInt aprilTagIdOptional = findNearestAprilTagForCurrentPose(2,3);
        if (aprilTagIdOptional.isEmpty()) {
            return Commands.none();
        }
        int aprilTagId = aprilTagIdOptional.getAsInt();
        */
        int aprilTagId = (int)SmartDashboard.getNumber("NearestAprilTagFeederID",2);
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        driveToFeeder(aprilTagId, side),
                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                        new LowerCoralElevator(coralElevator)
                ),
                new CollectCoral(coralGripper)
        );
    }

    private Command driveToProcessorAndPlaceAlgae() {
        if(!algaeGripper.hasAlgae()){
            return Commands.none();
        }

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                driveToProcessor(),
                new LowerCoralElevator(coralElevator)),
                algaeOut()
        );
    }

    private Command coralPut() {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                Commands.waitUntil(() -> coralArmCommand.didReachTargetPosition()),
                new ReleaseCoral(coralGripper)

        );
    }

    private Command coralCollect() {
        return new ParallelCommandGroup(
                new CollectCoral(coralGripper),
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                new LowerCoralElevator(coralElevator)
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

    public Command driveToNearestReefAndPut(ReefStandRow row, boolean level3) {
       /* OptionalInt aprilTagIdOptional = findNearestAprilTagForCurrentPose(0,1);
        if (aprilTagIdOptional.isEmpty()) {
            return Commands.none();
        }
        int aprilTagId = aprilTagIdOptional.getAsInt();
        */
        if(!coralGripper.hasCoral()){
            return Commands.none();
        }
        int aprilTagId = (int)SmartDashboard.getNumber("NearestAprilTagID",8);
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                    Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                    driveToReef(aprilTagId,row),
                    new SequentialCommandGroup(
                    new DistanceDelay(visionSystem,swerve,aprilTagId),
                    level3 ? new RaiseCoralElevator(coralElevator) : Commands.none()
                    )
                ),
        new ReleaseCoral(coralGripper)
        );
    }

    public Command driveToReef(int aprilTagId, ReefStandRow row) {
        Pose2d pose = visionSystem.getPoseForReefStand(aprilTagId, row);
        return driveToPose(pose);
    }

    public Command driveToProcessor() {
        int aprilTagId = isRed() ? 3 : 16;
        Pose2d pose = visionSystem.getPoseToProcessor(aprilTagId);
        return driveToPose(pose);
    }

    public Command driveToFeeder(int aprilTagId, FeederSide side) {
        Pose2d pose = visionSystem.getPoseForFeeder(aprilTagId, side);
        // rotate 180 because we want our back to the feeder
        Pose2d rotated = new Pose2d(pose.getX(), pose.getY(), pose.getRotation().rotateBy(Rotation2d.k180deg));
        return driveToPose(rotated);
    }

    private Command driveToPose(Pose2d pose) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> swerve.getField().getObject("Target").setPose(pose)),
                AutoBuilder.pathfindToPose(pose, RobotMap.CONSTRAINTS),
                Commands.runOnce(() -> swerve.getField().getObject("Target").setPoses())
        );
    }

    public OptionalInt findNearestAprilTagForCurrentPose(int red, int blue) {
        int sideIndex = isRed() ? red : blue;
        return findNearestAprilTag( RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE[sideIndex]);
    }

    private OptionalInt findNearestAprilTag(int[] aprilTagIds) {
        Pose2d pose = swerve.getPose();
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

    private boolean isRed(){
        Optional<DriverStation.Alliance> optionalAlliance = DriverStation.getAlliance();
        DriverStation.Alliance alliance = optionalAlliance.orElse(null);
        return alliance == DriverStation.Alliance.Red;
    }

    private boolean isFound(int[]arr,int num){
        for (int n : arr) {
            if(n == num) return true;
        }
        return false;
    }



    /*
        private Command twoCoralsAutoLeftRightSpecialReef8Feeder2() {
            return new SequentialCommandGroup(
                    driveToCoralReef8SpecialDownAndPlaceCoral(ReefStandRow.RIGHT, true),
                    Commands.runOnce(() -> DriverStation.reportWarning("Go To Feeder", false)),
                    driveAndCollectFromFeeder(2, FeederSide.CENTER),
                    Commands.runOnce(() -> DriverStation.reportWarning("Bitch Please", false)),
                    driveToCoralReefDownAndPlaceCoral(8, ReefStandRow.LEFT, true)
            );
        }
     */
}