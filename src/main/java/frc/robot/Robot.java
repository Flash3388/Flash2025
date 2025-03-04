package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.*;

import static edu.wpi.first.units.Units.Second;

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
    private boolean isGoingToFeeder = false;
    int dir;
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
        dir = 1;
        swerve.setDefaultCommand(swerve.driveA(
                () -> MathUtil.applyDeadband(-xboxMain.getLeftY() * dir, 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getLeftX() * dir, 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getRightX(), 0.05)
        ));

        configureButtons();
        SmartDashboard.putBoolean("isAuto", true);
        CommandScheduler.getInstance().setActiveButtonLoop(autoEventLoop);

        feederAuto = new SendableChooser<>();
        feederAuto.setDefaultOption("center", "CENTER");
        feederAuto.addOption("left", "LEFT");
        feederAuto.addOption("right", "RIGHT");
        SmartDashboard.putData("feederAutomation", feederAuto);
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("twoHighCoralsLeft", Commands.defer(()-> twoHighCoral(true),Set.of(swerve)));
        autoChooser.addOption("twoHighCoralsRight",Commands.defer(()-> twoHighCoral(false),Set.of(swerve)));
        autoChooser.addOption("orbitAuto", Commands.defer(()->orbitAuto(),Set.of()));
        autoChooser.addOption("oneHighAndTwoLowCoralsLeft", Commands.defer(()->oneHighAndTwoLow(true),Set.of(swerve)));
        autoChooser.addOption("oneHighAndTwoLowCoralsRight",Commands.defer(()-> oneHighAndTwoLow(false),Set.of(swerve)));
        autoChooser.addOption("twoHighCoralsAndAlgaeLeft", Commands.defer(()-> algaeAuto(true),Set.of(swerve)));
        autoChooser.addOption("twoHighCoralsAndAlgaeRight", Commands.defer(()-> algaeAuto(false),Set.of(swerve)));
        autoChooser.addOption("oneHighOneLowAndAlgaeLeft", Commands.defer(()-> oneHighOneLowAndAlgae(true),Set.of(swerve)));
        autoChooser.addOption("oneHighOneLowAndAlgaeRight", Commands.defer(()-> oneHighOneLowAndAlgae(false),Set.of(swerve)));
        SmartDashboard.putData("autoChooser", autoChooser);
        SmartDashboard.putBoolean("isLeft", true);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        visionSystem.changePipeLine(0);

        PathfindingCommand.warmupCommand().schedule();

        CommandScheduler.getInstance().onCommandInitialize((command)-> {
            System.out.printf("COMMAND %s:%s INIT\n", command.getName(), command.getClass().getSimpleName());
        });
        CommandScheduler.getInstance().onCommandFinish((command)-> {
            System.out.printf("COMMAND %s:%s END\n", command.getName(), command.getClass().getSimpleName());
        });
        CommandScheduler.getInstance().onCommandInterrupt((command)-> {
            System.out.printf("COMMAND %s:%s END\n", command.getName(), command.getClass().getSimpleName());
        });
    }

    Optional<LimelightHelpers.PoseEstimate> poseEstimate = Optional.empty();
    LimelightHelpers.PoseEstimate pose;

    @Override
    public void robotPeriodic() {
        //SmartDashboard.putNumber("pressure", compressor.getPressure());

        if (isGoingToFeeder) {
            newPattern = LEDPattern.solid(Color.kRed).breathe(Second.of(1));
        } else if (coralGripper.hasCoral() && algaeGripper.hasAlgae()) {
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

        boolean isAuto1 = SmartDashboard.getBoolean("isAuto", true);
        if (isAuto1 != isAuto) {
            isAuto = isAuto1;
            CommandScheduler.getInstance().setActiveButtonLoop(isAuto ? autoEventLoop : manualEventLoop);
        }

        poseEstimate = visionSystem.getRobotPoseEstimate();
        if (poseEstimate.isPresent()) {
            pose = poseEstimate.get();
            swerve.updatePoseEstimator(pose);
        }
        boolean isRed = isRed();
        dir = isRed ? -1 : 1;

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
        this.autoCommand = autoChooser.getSelected();;
        if(this.autoCommand  != null) {
            this.autoCommand.schedule();
        }

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }


    private Command orbitAuto() {
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0 : 1;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        reefAuto(ReefStandRow.RIGHT, aprilTags[sideIndexReef][5], true),
                        algaeCollect()),
                ProcessorAuto()
        );
    }

    public Command oneHighOneLowAndAlgae(boolean isLeft) {
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0 : 1;
        int sideIndexFeeder = isRed() ? 2 : 3;
        int indexReef = isLeft ? 1 : 3;
        int indexFeeder = isLeft ? 1 : 0;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        reefAuto(ReefStandRow.RIGHT, aprilTags[sideIndexReef][indexReef], true),
                        algaeCollect()),
                feederAuto(FeederSide.CENTER, aprilTags[sideIndexFeeder][indexFeeder]),
                reefAuto(ReefStandRow.RIGHT, aprilTags[sideIndexReef][indexReef], false),
                ProcessorAuto()
        );
    }

    private Command oneHighAndTwoLow(boolean isLeft) {
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0 : 1;
        int sideIndexFeeder = isRed() ? 2 : 3;
        int indexReef = isLeft ? 1 : 3;
        int indexFeeder = isLeft ? 1 : 0;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        reefAuto(ReefStandRow.RIGHT, aprilTags[sideIndexReef][indexReef], true),
                        algaeCollect()),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()), aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.LEFT, aprilTags[sideIndexReef][indexReef], false),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()), aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.RIGHT, aprilTags[sideIndexReef][indexReef], false)
        );
    }

    private Command twoHighCoral(boolean isLeft) {
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0 : 1;
        int sideIndexFeeder = isRed() ? 2 : 3;
        int indexReef = isLeft ? 1 : 3;
        int indexFeeder = isLeft ? 1 : 0;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        reefAuto(ReefStandRow.RIGHT, aprilTags[sideIndexReef][indexReef], true),
                        algaeCollect()),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()), aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.LEFT, aprilTags[sideIndexReef][indexReef], true),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()), aprilTags[sideIndexFeeder][indexFeeder])
        );
    }

    private Command algaeAuto(boolean isLeft) {
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0 : 1;
        int sideIndexFeeder = isRed() ? 2 : 3;
        int indexReef = isLeft ? 1 : 3;
        int indexFeeder = isLeft ? 1 : 0;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        reefAuto(ReefStandRow.RIGHT, aprilTags[sideIndexReef][indexReef], true),
                        algaeCollect()),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()), aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.LEFT, aprilTags[sideIndexReef][indexReef], true),
                ProcessorAuto()
        );
    }

    private Command ProcessorAuto() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        driveToProcessor(),
                        new LowerCoralElevator(coralElevator)),
                algaeOut()
        );
    }

    private Command feederAuto(FeederSide side, int aprilTagId) {
        return new ParallelCommandGroup(
                driveToFeeder(aprilTagId, side),
                new LowerCoralElevator(coralElevator),
                new SequentialCommandGroup(
                        new DistanceDelay(visionSystem, swerve, aprilTagId, 1.5),
                        new ParallelCommandGroup(
                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                        new CollectCoral(coralGripper)
                )));
    }

    private Command reefAuto(ReefStandRow row, int aprilTagId, boolean level3) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        driveToReef(aprilTagId, row),
                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                        new SequentialCommandGroup(
                                new DistanceDelay(visionSystem, swerve, aprilTagId, 3),
                                level3 ? new RaiseCoralElevator(coralElevator) : Commands.none())),
                new ReleaseCoral(coralGripper)
        );
    }


    private Command driveToProcessorAndPlaceAlgae() {
        if (!algaeGripper.hasAlgae()) {
            return Commands.none();
        }

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        driveToProcessor(),
                        new LowerCoralElevator(coralElevator)),
                algaeOut()
        );
    }

    public Command driveToNearestL1Reef(){
        OptionalInt optionalAprilTag = findNearestAprilTagForCurrentPose(0,1);
        if(optionalAprilTag.isEmpty()){
            DriverStation.reportWarning("No nearest AprilTag", true);
            return Commands.none();
        }
        int aprilTagId = optionalAprilTag.getAsInt();
        System.out.println("Going to AprilTag: " + aprilTagId);
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        goToReefAndPutToL1(aprilTagId),
                        new LowerCoralElevator(coralElevator),
                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A))
                ),
                Commands.waitUntil(()->coralArmCommand.didReachTargetPosition()),
                new ReleaseCoral(coralGripper)
        );
    }

    public Command goToReefAndPutToL1(int id){
        double offsetRight =0.45-RobotMap.OFFSET_REEF;
        double offsetAngle =7.7;

        Pose2d pose = visionSystem.getPoseForReefStandWithOffset(id,ReefStandRow.RIGHT,offsetRight);
        double rotDegrees = pose.getRotation().getDegrees()+offsetAngle;
        Pose2d targetPose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(rotDegrees));
        return driveToPose(targetPose);
    }

    public Command driveAndCollectAlgae() {
        return Commands.defer(
                this::goAndCollectAlgae, Set.of(swerve)
        );
    }

    private Command goAndCollectAlgae() {
        if (algaeGripper.hasAlgae()) {
            return Commands.none();
        }
        OptionalInt OptionalAprilTagId = findNearestAprilTagForCurrentPose(0, 1);
        if (OptionalAprilTagId.isEmpty()) {
            return Commands.none();
        }
        int aprilTagId = OptionalAprilTagId.getAsInt();
        return new ParallelCommandGroup(
                algaeCollect(),
                driveToReef(aprilTagId, null),
                new SequentialCommandGroup(
                        new DistanceDelay(visionSystem, swerve, aprilTagId, 3),
                        new RaiseCoralElevator(coralElevator)
                )
        );

    }

    private Command goAndCollectFromFeeder(FeederSide side) {

        OptionalInt OptionalAprilTagId = findNearestAprilTagForCurrentPose(2, 3);
        if (OptionalAprilTagId.isEmpty()) {
            return Commands.none();
        }
        int aprilTagId = OptionalAprilTagId.getAsInt();

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        driveToFeeder(aprilTagId, side),
                        new LowerCoralElevator(coralElevator),
                        new DistanceDelay(visionSystem, swerve, aprilTagId, 1.5)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                                        new SequentialCommandGroup(
                                                Commands.waitUntil(()-> coralArmCommand.didReachTargetPosition()),
                                                Commands.defer(() -> Commands.runOnce(() -> isGoingToFeeder = true), Set.of())
                                        ),
                                        new CollectCoral(coralGripper)
                                ),
                                new SequentialCommandGroup(
                                        Commands.waitUntil(() -> coralGripper.hasCoral()),
                                        Commands.defer(() -> Commands.runOnce(() -> isGoingToFeeder = false), Set.of())
                                )
                        ),
                        swerve.driveA(
                                () -> MathUtil.applyDeadband(-xboxMain.getLeftY() * dir, 0.05),
                                () -> MathUtil.applyDeadband(-xboxMain.getLeftX() * dir, 0.05),
                                () -> MathUtil.applyDeadband(-xboxMain.getRightX(), 0.05)
                        )
                )
        );
    }

    public Command goToNearestReefAndPut(ReefStandRow row, boolean level3) {

        if(!coralGripper.hasCoral()) {
            return Commands.none();
        }
        OptionalInt OptionalAprilTagId = findNearestAprilTagForCurrentPose(0, 1);
        if (OptionalAprilTagId.isEmpty()) {
            return Commands.none();
        }
        int aprilTagId = OptionalAprilTagId.getAsInt();
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                        driveToReef(aprilTagId, row),
                        new SequentialCommandGroup(
                                new DistanceDelay(visionSystem, swerve, aprilTagId, 3),
                                level3 ? new RaiseCoralElevator(coralElevator) : Commands.none()
                        )
                ),
                //Commands.waitUntil(() -> coralArmCommand.didReachTargetPosition()),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                Commands.waitUntil(()-> xboxSecond.a().getAsBoolean()),
                                new ReleaseCoral(coralGripper)
                        ),
                        swerve.driveA(
                                () -> MathUtil.applyDeadband(-xboxMain.getLeftY() * dir, 0.05),
                                () -> MathUtil.applyDeadband(-xboxMain.getLeftX() * dir, 0.05),
                                () -> MathUtil.applyDeadband(-xboxMain.getRightX(), 0.05)
                        )
                )
        );
    }

    private Command algaeRemoveFromTop() {
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
                new CollectAlgae(algaeGripper,4.6)
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
        if (row == null) {
            pose = visionSystem.getPoseForReefAlgae(aprilTagId);
        } else {
            pose = visionSystem.getPoseForReefStand(aprilTagId, row);
        }
        return driveToPose(pose);
    }

    public Command driveToReefAndKnockAlgae() {
        OptionalInt OptionalAprilTagId = findNearestAprilTagForCurrentPose(0, 1);
        if (OptionalAprilTagId.isEmpty()) {
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
                Commands.runOnce(() -> {
                    System.out.println("Start drive to pose: " + pose);
                    swerve.getField().getObject("Target").setPose(pose);
                }),
                AutoBuilder.pathfindToPose(pose, RobotMap.CONSTRAINTS),
                Commands.runOnce(() -> {
                    System.out.println("End drive to pose");
                    swerve.getField().getObject("Target").setPoses();
                })
        );
    }

    public OptionalInt findNearestAprilTagForCurrentPose(int red, int blue) {
        int sideIndex = isRed() ? red : blue;
        return findNearestAprilTag(RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE[sideIndex]);
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

    private boolean isRed() {
        Optional<DriverStation.Alliance> optionalAlliance = DriverStation.getAlliance();
        DriverStation.Alliance alliance = optionalAlliance.orElse(null);
        if (alliance == null) {
            return true;
        }
        return alliance == DriverStation.Alliance.Red;
    }

    public void configureButtons() {
        // xboxSecond.a(manualEventLoop).onTrue(algaeRemoveFromTop());
        //xboxSecond.a(manualEventLoop).onTrue(new RaiseCoralElevator(coralElevator));
        //xboxSecond.b(manualEventLoop).onTrue(new LowerCoralElevator(coralElevator));

        xboxSecond.y(manualEventLoop).onTrue(new RaiseCoralElevator(coralElevator));
        xboxSecond.b(manualEventLoop).onTrue(coralPut());
        xboxSecond.x(manualEventLoop).onTrue(coralCollect());
        xboxSecond.leftBumper(manualEventLoop).onTrue(Commands.none());
        xboxSecond.rightBumper(manualEventLoop).onTrue(Commands.none());
        xboxSecond.pov(0, 45, manualEventLoop).onTrue(new ExtendedAlgaeArm(algaeArm));
        xboxSecond.pov(0, 135, manualEventLoop).onTrue(algaeOut());
        xboxSecond.pov(0, 225, manualEventLoop).onTrue(new RetractAlgaeArm(algaeArm));
        xboxSecond.pov(0, 315, manualEventLoop).onTrue(algaeCollect());

        xboxSecond.y(autoEventLoop).onTrue(Commands.defer(() -> goAndCollectFromFeeder(FeederSide.CENTER), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.b(autoEventLoop).onTrue(Commands.defer(() -> goAndCollectFromFeeder(FeederSide.RIGHT), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.x(autoEventLoop).onTrue(Commands.defer(() -> goAndCollectFromFeeder(FeederSide.LEFT), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.leftBumper(autoEventLoop).onTrue(Commands.defer(() -> driveAndCollectAlgae(), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.rightBumper(autoEventLoop).onTrue(Commands.defer(() -> driveToProcessorAndPlaceAlgae(), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.pov(0, 45, autoEventLoop).onTrue(Commands.defer(() -> goToNearestReefAndPut(ReefStandRow.RIGHT, true), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.pov(0, 135, autoEventLoop).onTrue(Commands.defer(() -> goToNearestReefAndPut(ReefStandRow.RIGHT, false), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.pov(0, 225, autoEventLoop).onTrue(Commands.defer(() -> goToNearestReefAndPut(ReefStandRow.LEFT, false), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.pov(0, 315, autoEventLoop).onTrue(Commands.defer(() -> goToNearestReefAndPut(ReefStandRow.LEFT, true), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));
        xboxSecond.leftStick(autoEventLoop).onTrue(Commands.runOnce(()-> {
            isAuto = false;
            CommandScheduler.getInstance().setActiveButtonLoop(isAuto ? autoEventLoop : manualEventLoop);
            SmartDashboard.putBoolean("isAuto",false);
        }));

        xboxSecond.leftStick(manualEventLoop).onTrue(Commands.runOnce(()-> {
            isAuto = true;
            CommandScheduler.getInstance().setActiveButtonLoop(isAuto ? autoEventLoop : manualEventLoop);
            SmartDashboard.putBoolean("isAuto",true);
        }));

        xboxSecond.rightStick(autoEventLoop).onTrue(Commands.defer(()->driveToNearestL1Reef(),Set.of(swerve, coralElevator, coralGripper)));

        //  xboxSecond.a(autoEventLoop).onTrue(Commands.defer(() -> driveToReefAndKnockAlgae(), Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper)));

        xboxMain.rightBumper(manualEventLoop).onTrue(new LowerCoralElevator(coralElevator));
        xboxMain.rightBumper(autoEventLoop).onTrue(new LowerCoralElevator(coralElevator));
        xboxMain.leftBumper(manualEventLoop).whileTrue(swerve.driveA(
                () -> MathUtil.applyDeadband(-xboxMain.getLeftY() * dir, 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getLeftX() * dir, 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getRightX(), 0.05)
        ));
        xboxMain.leftBumper(autoEventLoop).whileTrue(swerve.driveA(
                () -> MathUtil.applyDeadband(-xboxMain.getLeftY() * dir, 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getLeftX() * dir, 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getRightX(), 0.05)
        ));
    }
}