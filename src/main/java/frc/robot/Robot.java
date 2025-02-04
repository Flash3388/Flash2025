package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.CoralArmCommand;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.CoralGripper;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeGripper;
import frc.robot.subsystems.CoralElevator;
import java.util.Set;

public class Robot extends TimedRobot {
    private final int MIN_PRESSURE = 80;
    private final int MAX_PRESSURE = 120;
    private AlgaeArm algaeArm;
    private AlgaeGripper algaeGripper;
    private CoralElevator coralElevator;
    private XboxController xbox;
    private CoralGripper coralGripper;
    private CoralArm coralArm;
    private Dashboard dashboard;
    private PneumaticHub ph;
    private CoralArmCommand coralArmCommand;
    private Compressor cpr;


    @Override
    public void robotInit() {
        algaeArm = new AlgaeArm();
        algaeGripper = new AlgaeGripper();
        coralElevator = new CoralElevator();
        coralGripper = new CoralGripper();
        coralArm = new CoralArm();
        xbox = new XboxController(0);
        ph = new PneumaticHub();
        cpr = new Compressor(RobotMap.COMPRESSION_PORT, PneumaticsModuleType.REVPH);
        cpr.enableAnalog(MIN_PRESSURE, MAX_PRESSURE);
        ph.enableCompressorAnalog(MIN_PRESSURE,MAX_PRESSURE);
        ph.enableCompressorDigital();
        dashboard = new Dashboard(algaeArm,algaeGripper,coralElevator,coralArm,coralGripper);
        coralArmCommand = new CoralArmCommand(coralArm);
        coralArm.setDefaultCommand(coralArmCommand);

        Command checkIfAlgaeRetract = Commands.defer(()->{
            if(algaeArm.isExtended()){
                return new RetractAlgaeArm(algaeArm);
            }
            return Commands.idle(algaeArm);
        }, Set.of(algaeArm));
        algaeArm.setDefaultCommand(checkIfAlgaeRetract);

        Command checkIfLow = Commands.defer(()-> {
            if(coralElevator.isRaised()){
                return new LowerCoralElevator(coralElevator);
            }
            return Commands.idle(coralElevator);
        }, Set.of(coralElevator));
        coralElevator.setDefaultCommand(checkIfLow);

        Command checkCoral = Commands.defer(()-> {
            if (coralGripper.hasCoral()) {
                return new HoldCoral(coralGripper);
            }
            return Commands.idle(coralGripper);
        }, Set.of(coralGripper));
        coralGripper.setDefaultCommand(checkCoral);

        Command checkAlgae = Commands.defer(()-> {
            if (algaeGripper.hasAlgae()) {
                return new HoldAlgae(algaeGripper);
            }
            return Commands.idle(algaeGripper);
        }, Set.of(algaeGripper));
        algaeGripper.setDefaultCommand(checkAlgae);
        new JoystickButton(xbox, XboxController.Button.kY.value)
                .whileTrue(new CollectCoral(coralGripper));
        new JoystickButton(xbox, XboxController.Button.kA.value)
                .whileTrue(new ReleaseCoral(coralGripper));
        new JoystickButton(xbox, XboxController.Button.kX.value)
                .whileTrue(new CollectAlgae(algaeGripper));
        new JoystickButton(xbox, XboxController.Button.kB.value)
                .whileTrue(new ReleaseAlgae(algaeGripper));

    }

    @Override
    public void robotPeriodic() {/*
        Compressor cpr = ph.makeCompressor();
        ph.enableCompressorAnalog(minPresseure,maxPresseure);*/
        SmartDashboard.putNumber("pressure",ph.getPressure(0));
        ph.enableCompressorAnalog(MIN_PRESSURE,MAX_PRESSURE);
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
        cpr.enableDigital();
    }

    @Override
    public void teleopPeriodic() {/*
        if (xbox.getAButtonPressed()) {
            coralElevator.piston1.set(DoubleSolenoid.Value.kForward);
            System.out.println("Piston Forward!");
        }
        if (xbox.getBButtonPressed()) {
            coralElevator.piston1.set(DoubleSolenoid.Value.kReverse);
            System.out.println("Piston Reverse!");
        }
        */
    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

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
                new CollectCoral(coralGripper));
    }

    private Command coralLevel3Place(){
        return new SequentialCommandGroup(
                Commands.runOnce(()-> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                new ParallelCommandGroup(
                        new RaiseCoralElevator(coralElevator),
                        Commands.waitUntil(()-> coralArmCommand.didReachTargetPosition())
                ),
                new CollectCoral(coralGripper));
    }

    private Command coralCollect(){
        return new SequentialCommandGroup(
                Commands.runOnce(()-> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                new ParallelCommandGroup(
                        new LowerCoralElevator(coralElevator),
                        Commands.waitUntil(()-> coralArmCommand.didReachTargetPosition())
                ),
                new ReleaseCoral(coralGripper));
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
