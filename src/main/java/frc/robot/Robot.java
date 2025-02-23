package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private Compressor compressor;
    private LedLights leds;

    private CoralArmCommand coralArmCommand;

    private CommandXboxController xboxSecond;
    private CommandXboxController xboxMain;
    private EventLoop autoEventLoop;
    private EventLoop manualEventLoop;

    private LEDPattern newPattern;
    private boolean isAuto = true;
    private SendableChooser<String> feederAuto;
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

        xboxSecond = new CommandXboxController(0);
        xboxMain = new CommandXboxController(1);
        autoEventLoop = new EventLoop();
        manualEventLoop = new EventLoop();

        compressor = new Compressor(RobotMap.COMPRESSION_PORT, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(RobotMap.MIN_PRESSURE, RobotMap.MAX_PRESSURE);

        leds = new LedLights();

        coralArmCommand = new CoralArmCommand(coralArm);
        coralArm.setDefaultCommand(coralArmCommand);
        boolean isRed = isRed();
        int dir = isRed? -1:1;
        swerve.setDefaultCommand(swerve.driveA(
                () -> MathUtil.applyDeadband(-xboxMain.getLeftY()*dir, 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getLeftX()*dir, 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getRightX(), 0.05)
        ));

        configureButtons();
        SmartDashboard.putBoolean("isAuto", true);
        CommandScheduler.getInstance().setActiveButtonLoop(autoEventLoop);

        feederAuto = new SendableChooser<>();
        feederAuto.setDefaultOption("center","CENTER");
        feederAuto.addOption("left","LEFT");
        feederAuto.addOption("right","RIGHT");
        SmartDashboard.putData("feederAutomation",feederAuto);
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("twoHighCoralsLeft",twoHighCoral(true));
        autoChooser.addOption("twoHighCoralsRight",twoHighCoral(false));
        autoChooser.addOption("orbitAuto",orbitAuto());
        autoChooser.addOption("oneHighAndTwoLowCoralsLeft",oneHighAndTwoLow(true));
        autoChooser.addOption("oneHighAndTwoLowCoralsRight",oneHighAndTwoLow(false));
        autoChooser.addOption("twoHighCoralsAndAlgaeLeft",algaeAuto(true));
        autoChooser.addOption("twoHighCoralsAndAlgaeRight",algaeAuto(false));
        autoChooser.addOption("oneHighOneLowAndAlgaeLeft",oneHighOneLowAndAlgae(true));
        autoChooser.addOption("oneHighOneLowAndAlgaeRight",oneHighOneLowAndAlgae(false));
        SmartDashboard.putData("autoChooser",autoChooser);
        SmartDashboard.putBoolean("isLeft",true);

        PathfindingCommand.warmupCommand().schedule();
    }

    Optional<LimelightHelpers.PoseEstimate> poseEstimate = Optional.empty();
    LimelightHelpers.PoseEstimate pose;

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("pressure",compressor.getPressure());
        if (coralGripper.hasCoral() && algaeGripper.hasAlgae()) {
            newPattern = LEDPattern.solid(Color.kTurquoise);
        } else if (algaeGripper.hasAlgae()) {
            newPattern = LEDPattern.solid(Color.kPurple);
        } else if (coralGripper.hasCoral()) {
            newPattern = LEDPattern.solid(Color.kWhite);
        } else {
            newPattern = leds.getFlashPattern();
        }

        if (!newPattern.equals(leds.getCurrentPattern())) {
            leds.setPattern(newPattern);
        }

        boolean isAuto1 = SmartDashboard.getBoolean("isAuto",true);
        if(isAuto1 != isAuto){
            isAuto = isAuto1;
            CommandScheduler.getInstance().setActiveButtonLoop(isAuto ? autoEventLoop : manualEventLoop);
        }

        poseEstimate= visionSystem.getRobotPoseEstimate();
        if(poseEstimate.isPresent()){
            pose = poseEstimate.get();
            swerve.updatePoseEstimator(pose);
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
        RobotMap.LIMELIGHT_DISTANCE_TO_TARGET_LIMIT = 4;
        visionSystem.changePipeLine(0);
    }

    @Override
    public void disabledPeriodic() {

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
        autoCommand = autoChooser.getSelected();
        autoCommand.schedule();

        compressor.disable();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {
        autoCommand.cancel();
        compressor.enableAnalog(RobotMap.MIN_PRESSURE, RobotMap.MAX_PRESSURE);
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

    public Command oneHighOneLowAndAlgae(boolean isLeft){
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0:1;
        int sideIndexFeeder = isRed() ? 2:3;
        int indexReef = isLeft ? 1:3;
        int indexFeeder = isLeft? 1 : 0;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                reefAuto(ReefStandRow.RIGHT,aprilTags[sideIndexReef][indexReef],true),
                algaeCollect()),
                feederAuto(FeederSide.CENTER,aprilTags[sideIndexFeeder][indexFeeder]),
                reefAuto(ReefStandRow.RIGHT,aprilTags[sideIndexReef][indexReef],false),
                ProcessorAuto()
        );
    }

    private Command oneHighAndTwoLow(boolean isLeft){
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
                reefAuto(ReefStandRow.LEFT,aprilTags[sideIndexReef][indexReef],false),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()),aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.RIGHT,aprilTags[sideIndexReef][indexReef],false)
        );
    }

    private Command twoHighCoral(boolean isLeft){
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0:1;
        int sideIndexFeeder = isRed() ? 2:3;
        int indexReef = isLeft ? 1:3;
        int indexFeeder = isLeft? 1 : 0;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                reefAuto(ReefStandRow.RIGHT,aprilTags[sideIndexReef][indexReef],true)),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()),aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.LEFT,aprilTags[sideIndexReef][indexReef],true),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()),aprilTags[sideIndexFeeder][indexFeeder])
        );
    }

    private Command algaeAuto(boolean isLeft){
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
            new LowerCoralElevator(coralElevator),
            new CollectCoral(coralGripper),
            new SequentialCommandGroup(
            new DistanceDelay(visionSystem,swerve,aprilTagId,1.5),
            Commands.runOnce(()-> System.out.println("HEY23258964yk6t43fk")),
            Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B))
        ));
    }

    private Command reefAuto(ReefStandRow row, int aprilTagId, boolean level3) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                driveToReef(aprilTagId,row),
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                new SequentialCommandGroup(
                new DistanceDelay(visionSystem,swerve,aprilTagId,3),
                level3 ? new RaiseCoralElevator(coralElevator) : Commands.none())),
                new ReleaseCoral(coralGripper)
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

    public Command driveAndCollectFromFeeder(FeederSide side){
        return Commands.defer(
                ()-> goAndCollectFromFeeder(side),
                Set.of(swerve)
        );
    }

    public Command driveToNearestReefAndPut(ReefStandRow row, boolean level3){
        return goToNearestReefAndPut(row,level3);
    }

    public Command driveAndCollectAlgae(){
        return Commands.defer(
                this::goAndCollectAlgae,Set.of(swerve)
        );
    }

    private Command goAndCollectAlgae(){
        if(algaeGripper.hasAlgae()){
            return Commands.none();
        }
        OptionalInt OptionalAprilTagId = findNearestAprilTagForCurrentPose(0,1);
        if(OptionalAprilTagId.isEmpty()){
            return Commands.none();
        }
        int aprilTagId = OptionalAprilTagId.getAsInt();
        return new ParallelCommandGroup(
            algaeCollect(),
            driveToReef(aprilTagId,null),
            new SequentialCommandGroup(
                new DistanceDelay(visionSystem,swerve,aprilTagId,3),
                new RaiseCoralElevator(coralElevator)
            )
        );

    }

    private Command goAndCollectFromFeeder(FeederSide side) {

        OptionalInt OptionalAprilTagId = findNearestAprilTagForCurrentPose(2,3);
        if(OptionalAprilTagId.isEmpty()){
            return Commands.none();
        }
        int aprilTagId = OptionalAprilTagId.getAsInt();

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        driveToFeeder(aprilTagId, side),
                        new LowerCoralElevator(coralElevator)
                ),
                new CollectCoral(coralGripper),
                new DistanceDelay(visionSystem,swerve,aprilTagId,1.5),
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B))
        );
    }

    public Command goToNearestReefAndPut(ReefStandRow row, boolean level3) {

        if(!coralGripper.hasCoral()){
            return Commands.none();
        }
        OptionalInt OptionalAprilTagId = findNearestAprilTagForCurrentPose(0,1);
        if(OptionalAprilTagId.isEmpty()){
            return Commands.none();
        }
        int aprilTagId = OptionalAprilTagId.getAsInt();
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                        driveToReef(aprilTagId,row),
                        new SequentialCommandGroup(
                                new DistanceDelay(visionSystem,swerve,aprilTagId,3),
                                level3 ? new RaiseCoralElevator(coralElevator) : Commands.none()
                        )
                ),
                Commands.waitUntil(()-> coralArmCommand.didReachTargetPosition()),
                new ReleaseCoral(coralGripper)
        );
    }

    private Command algaeRemoveFromTop(){
        return new SequentialCommandGroup(
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_C)),
                Commands.waitUntil(() -> coralArmCommand.didReachTargetPosition()),
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B))
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
            new LowerCoralElevator(coralElevator),
            new CollectCoral(coralGripper),
            Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B))
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

    public Command driveToReef(int aprilTagId, ReefStandRow row) {
        Pose2d pose;
        if(row == null){
            pose = visionSystem.getPoseForReefAlgae(aprilTagId);
        }else {
            pose = visionSystem.getPoseForReefStand(aprilTagId, row);
        }
        return driveToPose(pose);
    }

    public Command driveToReefAndKnockAlgae(){
        OptionalInt OptionalAprilTagId = findNearestAprilTagForCurrentPose(0,1);
        if(OptionalAprilTagId.isEmpty()){
            return Commands.none();
        }
        int aprilTagId = OptionalAprilTagId.getAsInt();

        Pose2d pose = visionSystem.getPoseForReefStandWithOffset(aprilTagId, ReefStandRow.RIGHT, -0.11);

        return new ParallelCommandGroup(
                driveToPose(pose),
                algaeRemoveFromTop()
        );
    }

    public Command driveToProcessor() {
        int aprilTagId = isRed() ? 3 : 16;
        Pose2d pose = visionSystem.getPoseToProcessor(aprilTagId);
        return driveToPose(pose);
    }

    public Command driveToFeeder(int aprilTagId, FeederSide side) {
        Pose2d pose = visionSystem.getPoseForFeeder(aprilTagId, side);

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
        if(alliance == null){
            return isRed();
        }
        return alliance == DriverStation.Alliance.Red;
    }
    public void configureButtons(){
        xboxSecond.a(manualEventLoop).onTrue(algaeRemoveFromTop());
        xboxSecond.y(manualEventLoop).onTrue(new RaiseCoralElevator(coralElevator));
        xboxSecond.b(manualEventLoop).onTrue(coralPut());
        xboxSecond.x(manualEventLoop).onTrue(coralCollect());
        xboxSecond.leftBumper(manualEventLoop).onTrue(Commands.none());
        xboxSecond.rightBumper(manualEventLoop).onTrue(Commands.none());
        xboxSecond.pov(0, 45, manualEventLoop).onTrue(new ExtendedAlgaeArm(algaeArm));
        xboxSecond.pov(0, 135, manualEventLoop).onTrue(algaeOut());
        xboxSecond.pov(0, 225, manualEventLoop).onTrue(new RetractAlgaeArm(algaeArm));
        xboxSecond.pov(0, 315, manualEventLoop).onTrue(algaeCollect());

        xboxSecond.a(autoEventLoop).onTrue(Commands.defer(()-> driveToReefAndKnockAlgae(), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.y(autoEventLoop).onTrue(Commands.defer(()-> driveAndCollectFromFeeder(FeederSide.CENTER), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.b(autoEventLoop).onTrue(Commands.defer(()-> driveAndCollectFromFeeder(FeederSide.RIGHT), Set.of(swerve, algaeArm, coralArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.x(autoEventLoop).onTrue(Commands.defer(()-> driveAndCollectFromFeeder(FeederSide.LEFT), Set.of(swerve, algaeArm, coralArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.leftBumper(autoEventLoop).onTrue(Commands.defer(()-> driveAndCollectAlgae(), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.rightBumper(autoEventLoop).onTrue(Commands.defer(()-> driveToProcessorAndPlaceAlgae(), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.pov(0, 45, autoEventLoop).onTrue(Commands.defer(()-> goToNearestReefAndPut(ReefStandRow.RIGHT,true), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.pov(0, 135, autoEventLoop).onTrue(Commands.defer(()-> goToNearestReefAndPut(ReefStandRow.RIGHT,false), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.pov(0, 225, autoEventLoop).onTrue(Commands.defer(()-> goToNearestReefAndPut(ReefStandRow.LEFT,false), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.pov(0, 315, autoEventLoop).onTrue(Commands.defer(()-> goToNearestReefAndPut(ReefStandRow.LEFT,true), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));


        xboxMain.rightBumper(manualEventLoop).onTrue(new LowerCoralElevator(coralElevator));
        xboxMain.rightBumper(autoEventLoop).onTrue(new LowerCoralElevator(coralElevator));

        /*new JoystickButton(xboxSecond, XboxController.Button.kY.value).onTrue(yButton());
        new JoystickButton(xboxSecond, XboxController.Button.kA.value).onTrue(aButton());
        new JoystickButton(xboxSecond, XboxController.Button.kX.value).onTrue(xButton());
        new JoystickButton(xboxSecond, XboxController.Button.kB.value).onTrue(bButton());
        new JoystickButton(xboxSecond, XboxController.Button.kRightBumper.value).onTrue(rightBumperButton());
        new JoystickButton(xboxSecond, XboxController.Button.kLeftBumper.value).onTrue(leftBumperButton());
        new POVButton(xboxSecond,315).onTrue(POVButtonLeftUp());
        new POVButton(xboxSecond,135).onTrue(POVButtonRightDown());
        new POVButton(xboxSecond,45).onTrue(POVButtonRightUp());
        new POVButton(xboxSecond,225).onTrue(POVButtonLeftDown());*/

        //new JoystickButton(xboxMain,XboxController.Button.kRightBumper.value).onTrue(Commands.defer(()-> ,Set.of(coralElevator)));

     //   new JoystickButton(xboxMain,XboxController.Button.kLeftBumper.value).onTrue(leftBumperButton());
    }
}