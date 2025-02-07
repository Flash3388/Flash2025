package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import java.util.Timer;
import java.util.TimerTask;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.CollectAlgae;
import frc.robot.commands.CollectCoral;
import frc.robot.commands.CoralArmCommand;
import frc.robot.commands.ExtendedAlgaeArm;
import frc.robot.commands.HoldAlgae;
import frc.robot.commands.HoldCoral;
import frc.robot.commands.LowerCoralElevator;
import frc.robot.commands.RaiseCoralElevator;
import frc.robot.commands.ReleaseAlgae;
import frc.robot.commands.ReleaseCoral;
import frc.robot.commands.RetractAlgaeArm;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeGripper;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralGripper;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSystem;

import java.time.Instant;
import java.util.Optional;
import java.util.Set;

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

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        Command checkIfAlgaeRetract = Commands.defer(()->{
            if(algaeArm.isExtended()){
                return new RetractAlgaeArm(algaeArm);
            }
            return Commands.idle(algaeArm);
        }, Set.of(algaeArm));
        algaeArm.setDefaultCommand(checkIfAlgaeRetract);

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
                .onTrue(new RaiseCoralElevator(coralElevator));
        new JoystickButton(xbox, XboxController.Button.kA.value)
                .onTrue(new LowerCoralElevator(coralElevator));
        new JoystickButton(xbox, XboxController.Button.kX.value)
                .onTrue(new ExtendedAlgaeArm(algaeArm));
        new JoystickButton(xbox, XboxController.Button.kB.value)
                .onTrue(coralLevel2Place());
        new JoystickButton(xbox, XboxController.Button.kRightBumper.value)
                .onTrue(coralLevel3Place());
        new JoystickButton(xbox, XboxController.Button.kLeftBumper.value)
                .onTrue(coralCollect());
        new POVButton(xbox,180).onTrue(new CollectAlgae(algaeGripper));
        NamedCommands.registerCommand("dropL3",new ReleaseCoral(coralGripper));
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Pressure", pneumaticsHub.getPressure(0));

        Optional<LimelightHelpers.PoseEstimate> pose = visionSystem.getRobotPoseEstimate();
        //noinspection OptionalIsPresent
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
        compressor.enableAnalog(RobotMap.MIN_PRESSURE,RobotMap.MAX_PRESSURE);
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
        this.autoCommand = autoChooser.getSelected();
        if(this.autoCommand != null) {
            this.autoCommand.schedule();
        }

        // Create a timer to schedule the delayed execution
        Timer timer = new Timer();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                coralLevel3Place().schedule();
            }
        }, 8500); // Delay of 3000 milliseconds (3 seconds)
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

    private Command coralLevel2Place(){
        return new SequentialCommandGroup(
                Commands.runOnce(()-> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                new ParallelCommandGroup(
                        new LowerCoralElevator(coralElevator),
                        Commands.waitUntil(()-> coralArmCommand.didReachTargetPosition())
                ),
                new ReleaseCoral(coralGripper));
    }

    private Command coralLevel3Place(){
        return new SequentialCommandGroup(
                Commands.runOnce(()-> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                new ParallelCommandGroup(
                        new RaiseCoralElevator(coralElevator),
                        Commands.waitUntil(()-> coralArmCommand.didReachTargetPosition())
                ),
                new ReleaseCoral(coralGripper));
    }

    private Command coralCollect(){
        return new SequentialCommandGroup(
                Commands.runOnce(()-> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                new ParallelCommandGroup(
                        new LowerCoralElevator(coralElevator),
                        Commands.waitUntil(()-> coralArmCommand.didReachTargetPosition())
                ),
                new CollectCoral(coralGripper));
    }

    private Command algaeCollect(){
        return new SequentialCommandGroup(
                new ExtendedAlgaeArm(algaeArm),
                new CollectAlgae(algaeGripper));
    }

    private Command algaeOut(){
        return new SequentialCommandGroup(
                new RetractAlgaeArm(algaeArm),
                new ReleaseAlgae(algaeGripper));
    }
}
