package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.lang.reflect.Field;
import java.util.*;
import java.util.function.BooleanSupplier;

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

    private CommandXboxController xboxMain;

    private CommandGenericHID autoCommandsController;
    private CommandGenericHID manualCommandsController;
    private EventLoop redTeamLoop;
    private EventLoop blueTeamLoop;
    private ReefLevel nextSelectedLevel = ReefLevel.L1;

    private LEDPattern newPattern;
    private boolean isGoingToFeeder = false;
    private int swerveDriveDir;
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

        xboxMain = new CommandXboxController(1);
        autoCommandsController = new CommandGenericHID(4);
        manualCommandsController = new CommandGenericHID(5);
        redTeamLoop = new EventLoop();
        blueTeamLoop = new EventLoop();

        compressor = new Compressor(RobotMap.COMPRESSION_PORT, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(RobotMap.MIN_PRESSURE, RobotMap.MAX_PRESSURE);

        leds = new LedLights();

        coralArmCommand = new CoralArmCommand(coralArm);
        coralArm.setDefaultCommand(coralArmCommand);
        swerveDriveDir = 1;
        swerve.setDefaultCommand(createSwerveDriveCommand());

        // DON'T DELETE: FIX TO HIGH SMARTDASHBOARD USAGE
        try {
            Field field = SmartDashboard.class.getDeclaredField("tablesToData");
            field.setAccessible(true);
            Map<?, ?> tablesToData = (Map<?, ?>) field.get(null);

            for (Map.Entry<?, ?> entry : tablesToData.entrySet()) {
                System.out.printf("%s: %s, %s\n", entry.getKey().toString(), entry.getValue().toString(), entry.getValue().getClass().getSimpleName());
            }
            tablesToData.clear();
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }

        configureButtons();
        CommandScheduler.getInstance().setActiveButtonLoop(redTeamLoop);
        SmartDashboard.putString("AllianceMode", "Red");

        feederAuto = new SendableChooser<>();
        feederAuto.setDefaultOption("center", "CENTER");
        feederAuto.addOption("left", "LEFT");
        feederAuto.addOption("right", "RIGHT");
        SmartDashboard.putData("feederAutomation", feederAuto);
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("twoHighCoralsLeft", Commands.defer(()-> twoHighCoral(true),Set.of(swerve)));
        autoChooser.addOption("twoHighCoralsRight",Commands.defer(()-> twoHighCoral(false),Set.of(swerve)));
        autoChooser.addOption("orbitAuto", Commands.defer(()->orbitAuto(),Set.of()));
        autoChooser.addOption("twoLowCoralsLeft",Commands.defer(() -> twoLowCorals(true),Set.of(swerve)));
        autoChooser.addOption("twoLowCoralsRight",Commands.defer(() -> twoLowCorals(false),Set.of(swerve)));
        autoChooser.addOption("oneConveyorOneCoralLeft",Commands.defer(() -> oneConveyorOneLowCoral(true),Set.of(swerve)));
        autoChooser.addOption("oneConveyorOneCoralRight",Commands.defer(() -> oneConveyorOneLowCoral(false),Set.of(swerve)));
        autoChooser.addOption("oneHighAndTwoLowCoralsLeft", Commands.defer(()->oneHighAndTwoLow(true),Set.of(swerve)));
        autoChooser.addOption("oneHighAndTwoLowCoralsRight",Commands.defer(()-> oneHighAndTwoLow(false),Set.of(swerve)));
        autoChooser.addOption("twoHighCoralsAndAlgaeLeft", Commands.defer(()-> algaeAuto(true),Set.of(swerve)));
        autoChooser.addOption("twoHighCoralsAndAlgaeRight", Commands.defer(()-> algaeAuto(false),Set.of(swerve)));
        autoChooser.addOption("oneHighOneLowAndAlgaeLeft", Commands.defer(()-> oneHighOneLowAndAlgae(true),Set.of(swerve)));
        autoChooser.addOption("oneHighOneLowAndAlgaeRight", Commands.defer(()-> oneHighOneLowAndAlgae(false),Set.of(swerve)));
        SmartDashboard.putData("autoChooser", autoChooser);
        SmartDashboard.putBoolean("isLeft", true);

        visionSystem.changePipeLine(0);

        PathfindingCommand.warmupCommand().schedule();

        // TODO: REMOVE
//        CommandScheduler.getInstance().onCommandInitialize((command)-> {
//            System.out.printf("COMMAND INIT %s, %s\n", command.getName(), command.getClass().getSimpleName());
//        });
//        CommandScheduler.getInstance().onCommandInterrupt((command)-> {
//            System.out.printf("COMMAND INTERRUPT %s, %s\n", command.getName(), command.getClass().getSimpleName());
//        });
//        CommandScheduler.getInstance().onCommandFinish((command)-> {
//            System.out.printf("COMMAND FINISH %s, %s\n", command.getName(), command.getClass().getSimpleName());
//        });
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

        poseEstimate = visionSystem.getRobotPoseEstimate();
        if (poseEstimate.isPresent()) {
            pose = poseEstimate.get();
            swerve.updatePoseEstimator(pose);
        }

        boolean isRed = isRed();
        swerveDriveDir = isRed ? -1 : 1;

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
        visionSystem.changePipeLine(1);
        isGoingToFeeder = false;
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
        xboxMain.rightBumper().onTrue(new LowerCoralElevator(coralElevator));
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
        if (autoCommand != null) {
            autoCommand.schedule();
        }

        compressor.disable();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {
        if (autoCommand != null) {
            autoCommand.cancel();
            autoCommand = null;
        }

        compressor.enableAnalog(RobotMap.MIN_PRESSURE, RobotMap.MAX_PRESSURE);
    }

    @Override
    public void testInit() {
        Pose2d pose = new Pose2d(swerve.getPose().getX() + 3,swerve.getPose().getY(),swerve.getPose().getRotation());

        Commands.defer(() -> driveToPose(pose), Set.of(swerve)).schedule();
        swerve.getField().getObject("Target").setPose(pose);
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }

    private void configureButtons() {
        Command cancelCommand = Commands.runOnce(()->{}, swerve, algaeArm, coralElevator, coralGripper, algaeGripper);

        // red team
        autoCommandsController.button(1, redTeamLoop).onTrue(goAndCollectFromFeeder(1, FeederSide.RIGHT));
        autoCommandsController.button(2, redTeamLoop).onTrue(goAndCollectFromFeeder(1, FeederSide.CENTER));
        autoCommandsController.button(3, redTeamLoop).onTrue(goAndCollectFromFeeder(1, FeederSide.LEFT));
        autoCommandsController.button(4, redTeamLoop).onTrue(goAndCollectFromFeeder(2, FeederSide.LEFT));
        autoCommandsController.button(5, redTeamLoop).onTrue(goAndCollectFromFeeder(2, FeederSide.CENTER));
        autoCommandsController.button(6, redTeamLoop).onTrue(goAndCollectFromFeeder(2, FeederSide.RIGHT));
        autoCommandsController.button(7, redTeamLoop).onTrue(createCommandToProcessor(3));
        //autoCommandsController.button(8, redTeamLoop).onTrue(driveToParking(4));
        //autoCommandsController.button(9, redTeamLoop).onTrue(driveToParking(5));
        autoCommandsController.button(10, redTeamLoop).onTrue(createCommandToReef(6, ReefStandRow.LEFT));
        autoCommandsController.button(11, redTeamLoop).onTrue(createCommandToReef(6, ReefStandRow.RIGHT));
        autoCommandsController.button(12, redTeamLoop).onTrue(createCommandToReef(7, ReefStandRow.LEFT));
        autoCommandsController.button(13, redTeamLoop).onTrue(createCommandToReef(7, ReefStandRow.RIGHT));
        autoCommandsController.button(14, redTeamLoop).onTrue(createCommandToReef(8, ReefStandRow.LEFT));
        autoCommandsController.button(15, redTeamLoop).onTrue(createCommandToReef(8, ReefStandRow.RIGHT));
        autoCommandsController.button(16, redTeamLoop).onTrue(createCommandToReef(9, ReefStandRow.LEFT));
        autoCommandsController.button(17, redTeamLoop).onTrue(createCommandToReef(9, ReefStandRow.RIGHT));
        autoCommandsController.button(18, redTeamLoop).onTrue(createCommandToReef(10, ReefStandRow.LEFT));
        autoCommandsController.button(19, redTeamLoop).onTrue(createCommandToReef(10, ReefStandRow.RIGHT));
        autoCommandsController.button(20, redTeamLoop).onTrue(createCommandToReef(11, ReefStandRow.LEFT));
        autoCommandsController.button(21, redTeamLoop).onTrue(createCommandToReef(11, ReefStandRow.RIGHT));
        autoCommandsController.button(22, redTeamLoop).onTrue(Commands.runOnce(()-> {
            CommandScheduler.getInstance().setActiveButtonLoop(blueTeamLoop);
            SmartDashboard.putString("AllianceMode", "Blue");
        }));
        autoCommandsController.button(24, redTeamLoop).onTrue(Commands.runOnce(()-> nextSelectedLevel = ReefLevel.L1));
        autoCommandsController.button(25, redTeamLoop).onTrue(Commands.runOnce(()-> nextSelectedLevel = ReefLevel.L2));
        autoCommandsController.button(26, redTeamLoop).onTrue(Commands.runOnce(()-> nextSelectedLevel = ReefLevel.L3));
        autoCommandsController.button(27, redTeamLoop).onTrue(Commands.runOnce(()-> nextSelectedLevel = ReefLevel.L3ALGAE));

        // blue team
        autoCommandsController.button(1, blueTeamLoop).onTrue(goAndCollectFromFeeder(12, FeederSide.RIGHT));
        autoCommandsController.button(2, blueTeamLoop).onTrue(goAndCollectFromFeeder(12, FeederSide.CENTER));
        autoCommandsController.button(3, blueTeamLoop).onTrue(goAndCollectFromFeeder(12, FeederSide.LEFT));
        autoCommandsController.button(4, blueTeamLoop).onTrue(goAndCollectFromFeeder(13, FeederSide.LEFT));
        autoCommandsController.button(5, blueTeamLoop).onTrue(goAndCollectFromFeeder(13, FeederSide.CENTER));
        autoCommandsController.button(6, blueTeamLoop).onTrue(goAndCollectFromFeeder(13, FeederSide.RIGHT));
        autoCommandsController.button(7, blueTeamLoop).onTrue(createCommandToProcessor(16));
        //autoCommandsController.button(8, blueTeamLoop).onTrue(driveToParking(14));
        //autoCommandsController.button(9, blueTeamLoop).onTrue(driveToParking(15));
        autoCommandsController.button(10, blueTeamLoop).onTrue(createCommandToReef(17, ReefStandRow.LEFT));
        autoCommandsController.button(11, blueTeamLoop).onTrue(createCommandToReef(17, ReefStandRow.RIGHT));
        autoCommandsController.button(12, blueTeamLoop).onTrue(createCommandToReef(18, ReefStandRow.LEFT));
        autoCommandsController.button(13, blueTeamLoop).onTrue(createCommandToReef(18, ReefStandRow.RIGHT));
        autoCommandsController.button(14, blueTeamLoop).onTrue(createCommandToReef(19, ReefStandRow.LEFT));
        autoCommandsController.button(15, blueTeamLoop).onTrue(createCommandToReef(19, ReefStandRow.RIGHT));
        autoCommandsController.button(16, blueTeamLoop).onTrue(createCommandToReef(20, ReefStandRow.LEFT));
        autoCommandsController.button(17, blueTeamLoop).onTrue(createCommandToReef(20, ReefStandRow.RIGHT));
        autoCommandsController.button(18, blueTeamLoop).onTrue(createCommandToReef(21, ReefStandRow.LEFT));
        autoCommandsController.button(19, blueTeamLoop).onTrue(createCommandToReef(21, ReefStandRow.RIGHT));
        autoCommandsController.button(20, blueTeamLoop).onTrue(createCommandToReef(22, ReefStandRow.LEFT));
        autoCommandsController.button(21, blueTeamLoop).onTrue(createCommandToReef(22, ReefStandRow.RIGHT));
        autoCommandsController.button(23, blueTeamLoop).onTrue(Commands.runOnce(()-> {
            CommandScheduler.getInstance().setActiveButtonLoop(redTeamLoop);
            SmartDashboard.putString("AllianceMode", "Red");
        }));
        autoCommandsController.button(24, blueTeamLoop).onTrue(Commands.runOnce(()-> nextSelectedLevel = ReefLevel.L1));
        autoCommandsController.button(25, blueTeamLoop).onTrue(Commands.runOnce(()-> nextSelectedLevel = ReefLevel.L2));
        autoCommandsController.button(26, blueTeamLoop).onTrue(Commands.runOnce(()-> nextSelectedLevel = ReefLevel.L3));
        autoCommandsController.button(27, blueTeamLoop).onTrue(Commands.runOnce(()-> nextSelectedLevel = ReefLevel.L3ALGAE));

        // manual red
        manualCommandsController.button(1, redTeamLoop).onTrue(new RaiseCoralElevator(coralElevator));
        manualCommandsController.button(2, redTeamLoop).onTrue(new LowerCoralElevator(coralElevator));
        manualCommandsController.button(3, redTeamLoop).onTrue(new ExtendedAlgaeArm(algaeArm));
        manualCommandsController.button(4, redTeamLoop).onTrue(new RetractAlgaeArm(algaeArm));
        manualCommandsController.button(5, redTeamLoop).onTrue(new CollectAlgae(algaeGripper, 4.6));
        manualCommandsController.button(6, redTeamLoop).onTrue(new ReleaseAlgae(algaeGripper));
        manualCommandsController.button(7, redTeamLoop).onTrue(new CollectCoral(coralGripper));
        manualCommandsController.button(8, redTeamLoop).onTrue(new ReleaseCoral(coralGripper));
        manualCommandsController.button(9, redTeamLoop).onTrue(new SequentialCommandGroup(
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                Commands.waitUntil(() -> coralArmCommand.didReachTargetPosition())
        ));
        manualCommandsController.button(10, redTeamLoop).onTrue(new SequentialCommandGroup(
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                Commands.waitUntil(() -> coralArmCommand.didReachTargetPosition())
        ));

        manualCommandsController.button(11, redTeamLoop).onTrue(cancelCommand);
        // manual blue
        manualCommandsController.button(1, blueTeamLoop).onTrue(new RaiseCoralElevator(coralElevator));
        manualCommandsController.button(2, blueTeamLoop).onTrue(new LowerCoralElevator(coralElevator));
        manualCommandsController.button(3, blueTeamLoop).onTrue(new ExtendedAlgaeArm(algaeArm));
        manualCommandsController.button(4, blueTeamLoop).onTrue(new RetractAlgaeArm(algaeArm));
        manualCommandsController.button(5, blueTeamLoop).onTrue(new CollectAlgae(algaeGripper, 4.6));
        manualCommandsController.button(6, blueTeamLoop).onTrue(new ReleaseAlgae(algaeGripper));
        manualCommandsController.button(7, blueTeamLoop).onTrue(new CollectCoral(coralGripper));
        manualCommandsController.button(8, blueTeamLoop).onTrue(new ReleaseCoral(coralGripper));
        manualCommandsController.button(9, blueTeamLoop).onTrue(new SequentialCommandGroup(
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                Commands.waitUntil(() -> coralArmCommand.didReachTargetPosition())
        ));
        manualCommandsController.button(10, blueTeamLoop).onTrue(new SequentialCommandGroup(
                Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                Commands.waitUntil(() -> coralArmCommand.didReachTargetPosition())
        ));
        manualCommandsController.button(11, redTeamLoop).onTrue(cancelCommand);

        xboxMain.leftBumper(blueTeamLoop).whileTrue(createSwerveDriveCommand());
        xboxMain.leftBumper(redTeamLoop).whileTrue(createSwerveDriveCommand());
    }

    private Command orbitAuto() {
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0 : 1;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        reefAuto(ReefStandRow.RIGHT, aprilTags[sideIndexReef][5], true),
                        algaeCollect(4.6)),
                ProcessorAuto()
        );
    }

    private Command oneHighOneLowAndAlgae(boolean isLeft) {
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0 : 1;
        int sideIndexFeeder = isRed() ? 2 : 3;
        int indexReef = isLeft ? 1 : 3;
        int indexFeeder = isLeft ? 1 : 0;
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        reefAuto(ReefStandRow.RIGHT, aprilTags[sideIndexReef][indexReef], true),
                        algaeCollect(4.6)),
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
                        algaeCollect(4.6)),
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
                        algaeCollect(4.6)),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()), aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.LEFT, aprilTags[sideIndexReef][indexReef], true),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()), aprilTags[sideIndexFeeder][indexFeeder])
        );
    }

    private Command twoLowCorals(boolean isLeft) {
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0 : 1;
        int sideIndexFeeder = isRed() ? 2 : 3;
        int indexReef = isLeft ? 0 : 4;
        int indexFeeder = isLeft ? 1 : 0;
        return new SequentialCommandGroup(
                reefAuto(ReefStandRow.RIGHT, aprilTags[sideIndexReef][indexReef], false),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()), aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.LEFT, aprilTags[sideIndexReef][indexReef], false),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()), aprilTags[sideIndexFeeder][indexFeeder])
        );
    }

    private Command oneConveyorOneLowCoral(boolean isLeft) {
        int[][] aprilTags = RobotMap.REEF_APRIL_TAGS_BY_ALLIANCE;
        int sideIndexReef = isRed() ? 0 : 1;
        int sideIndexFeeder = isRed() ? 2 : 3;
        int indexReef = isLeft ? 0 : 4;
        int indexFeeder = isLeft ? 1 : 0;
        return new SequentialCommandGroup(
                reefAutoLow(aprilTags[sideIndexReef][indexReef]),
                Commands.none(),
                feederAuto(FeederSide.valueOf(feederAuto.getSelected()), aprilTags[sideIndexFeeder][indexFeeder]),
                Commands.none(),
                reefAuto(ReefStandRow.LEFT, aprilTags[sideIndexReef][indexReef], false),
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
                        algaeCollect(4.6)),
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

    private Command reefAutoLow(int aprilTagId) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        goToReefL1(aprilTagId),
                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A))
                ),
                new ReleaseCoral(coralGripper)
        );
    }

    private Command goToReefL1(int id){
        double offsetRight =1.225-RobotMap.OFFSET_REEF;
        double offsetOther =-0.22;
        double offsetAngle =73;

        Pose2d pose = visionSystem.getPoseForReefStandWithOffset(id,ReefStandRow.RIGHT,offsetRight,offsetOther);
        double rotDegrees = pose.getRotation().getDegrees()+offsetAngle;
        Pose2d targetPose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(rotDegrees));
        return driveToPose(targetPose);
    }

    private Command driveAndCollectAlgae() {
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
                algaeCollect(3),
                driveToReef(aprilTagId, null),
                new SequentialCommandGroup(
                        new DistanceDelay(visionSystem, swerve, aprilTagId, 3),
                        new RaiseCoralElevator(coralElevator)
                )
        );

    }

    private Command goAndCollectFromFeeder(int aprilTagId, FeederSide side) {
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
                        createSwerveDriveCommand()
                )
        );
    }

    private Command goToReefAndGetAlgae(int aprilTagId, ReefStandRow row) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        Commands.runOnce(() -> xboxMain.setRumble(GenericHID.RumbleType.kLeftRumble,0.3)),
                        driveToReef(aprilTagId, row),
                        new SequentialCommandGroup(
                                new DistanceDelay(visionSystem, swerve, aprilTagId, 3),
                                new RaiseCoralElevator(coralElevator)
                        ),
                        new ExtendedAlgaeArm(algaeArm)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new CollectAlgae(algaeGripper, 4),
                                Commands.runOnce(() -> xboxMain.setRumble(GenericHID.RumbleType.kBothRumble,0))
                        ),
                        Commands.runOnce(() -> xboxMain.setRumble(GenericHID.RumbleType.kLeftRumble,0)),
                        Commands.runOnce(() -> xboxMain.setRumble(GenericHID.RumbleType.kRightRumble,0.75)),
                        createSwerveDriveCommand()
                )
        );
    }

    private Command goToReefAndPut(int aprilTagId, ReefStandRow row, ReefLevel currentLevel, BooleanSupplier releaseCoral) {
        boolean level3 = currentLevel == ReefLevel.L3;
        boolean level1 = currentLevel == ReefLevel.L1;

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        Commands.runOnce(() -> xboxMain.setRumble(GenericHID.RumbleType.kLeftRumble,0.3)),
                        Commands.runOnce(() -> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                        level1 ? goToReefL1(aprilTagId) : driveToReef(aprilTagId, row),
                        new SequentialCommandGroup(
                                new DistanceDelay(visionSystem, swerve, aprilTagId, 3),
                                level3 ? new RaiseCoralElevator(coralElevator) : Commands.none()
                        )
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        level3 ? algaeCollect(2) : Commands.none(),
                                        new SequentialCommandGroup(
                                                Commands.waitUntil(releaseCoral),
                                                new ReleaseCoral(coralGripper),
                                                Commands.runOnce(() -> xboxMain.setRumble(GenericHID.RumbleType.kBothRumble,0))
                                        )
                                )
                        ),
                        Commands.runOnce(() -> xboxMain.setRumble(GenericHID.RumbleType.kLeftRumble,0)),
                        Commands.runOnce(() -> xboxMain.setRumble(GenericHID.RumbleType.kRightRumble,0.75)),
                        createSwerveDriveCommand()
                )
        );
    }

    private Command createCommandToReef(int aprilTagId, ReefStandRow row) {
        return Commands.defer(()-> {
            if(!coralGripper.hasCoral() && !(nextSelectedLevel == ReefLevel.L3ALGAE)) {
                return Commands.none();
            }

            return nextSelectedLevel == ReefLevel.L3ALGAE ? goToReefAndGetAlgae(aprilTagId, row) : goToReefAndPut(aprilTagId, row, nextSelectedLevel, ()-> xboxMain.getHID().getAButton());
        }, Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper));
    }

    private Command createCommandToProcessor(int aprilTagId) {
        return Commands.defer(()-> {
            if (!algaeGripper.hasAlgae()) {
                return Commands.none();
            }

            return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            driveToProcessor(aprilTagId),
                            new LowerCoralElevator(coralElevator)
                    ),
                    new ParallelDeadlineGroup(
                            new SequentialCommandGroup(
                                    Commands.waitUntil(()-> xboxMain.getHID().getAButton()),
                                    algaeOut()
                            ),
                            createSwerveDriveCommand()
                    )
            );
        }, Set.of(swerve, algaeArm, coralElevator, coralGripper, algaeGripper));
    }

    private Command driveToReefAndKnockAlgae() {
        OptionalInt OptionalAprilTagId = findNearestAprilTagForCurrentPose(0, 1);
        if (OptionalAprilTagId.isEmpty()) {
            return Commands.none();
        }
        int aprilTagId = OptionalAprilTagId.getAsInt();

        Pose2d pose = visionSystem.getPoseForReefStandWithOffset(aprilTagId, ReefStandRow.RIGHT, -0.11,0);

        return new ParallelCommandGroup(
                driveToPose(pose),
                algaeRemoveFromTop()
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

    private Command algaeCollect(double waitTime) {
        return new SequentialCommandGroup(
                new ExtendedAlgaeArm(algaeArm),
                new CollectAlgae(algaeGripper,waitTime)
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

    private Command driveToProcessor() {
        int aprilTagId = isRed() ? 3 : 16;
        return driveToProcessor(aprilTagId);
    }

    private Command driveToProcessor(int aprilTagId) {
        Pose2d pose = visionSystem.getPoseToProcessor(aprilTagId);
        return driveToPose(pose);
    }

    private Command driveToFeeder(int aprilTagId, FeederSide side) {
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

    private Command createSwerveDriveCommand() {
        return swerve.driveA(
                () -> MathUtil.applyDeadband(-xboxMain.getLeftY() * swerveDriveDir, 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getLeftX() * swerveDriveDir, 0.05),
                () -> MathUtil.applyDeadband(-xboxMain.getRightX(), 0.05)
        );
    }
}