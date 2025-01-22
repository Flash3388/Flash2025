package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.CoralGripper;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeGripper;
import frc.robot.subsystems.CoralElevator;

import java.util.Set;

public class Robot extends TimedRobot {
    private AlgaeArm algaeArm;
    private AlgaeGripper algaeGripper;
    private CoralElevator coralElevator;
    private CoralGripper coralGripper;
    private CoralArm coralArm;

    private CoralArmCommand coralArmCommand;
    private RetractAlgaeArm retractAlgaeArm;
    private LowerCoralElevator lowerCoralElevator;

    @Override
    public void robotInit() {
        algaeArm = new AlgaeArm();
        algaeGripper = new AlgaeGripper();
        coralElevator = new CoralElevator();
        coralGripper = new CoralGripper();
        coralArm = new CoralArm();

        coralArmCommand = new CoralArmCommand(coralArm);
        coralArm.setDefaultCommand(coralArmCommand);

        retractAlgaeArm = new RetractAlgaeArm(algaeArm);
        algaeArm.setDefaultCommand(retractAlgaeArm);

        lowerCoralElevator = new LowerCoralElevator(coralElevator);
        coralElevator.setDefaultCommand(lowerCoralElevator);

        Command CheckCoral = Commands.defer(()-> {
            if (coralGripper.hasCoral()) {
                return new HoldCoral(coralGripper);
            }
            return Commands.none();
        }, (Set<Subsystem>) coralGripper);
        coralGripper.setDefaultCommand(CheckCoral);

        Command CheckAlgae = Commands.defer(()-> {
            if (algaeGripper.hasAlgae()) {
                return new HoldAlgae(algaeGripper);
            }
            return Commands.none();
        }, (Set<Subsystem>) algaeGripper);
        algaeGripper.setDefaultCommand(CheckAlgae);

    }

    @Override
    public void robotPeriodic() {
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
                Commands.runOnce(()-> new LowerCoralElevator(coralElevator)),
                Commands.runOnce(()-> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                Commands.waitUntil(()-> coralArmCommand.didReachTargetPosition()),
                new CollectCoral(coralGripper));
    }

    private Command coralLevel3Place(){
        return new SequentialCommandGroup(
                Commands.runOnce(()-> new RaiseCoralElevator(coralElevator)),
                Commands.runOnce(()-> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_A)),
                Commands.waitUntil(()-> coralArmCommand.didReachTargetPosition()),
                new CollectCoral(coralGripper));
    }

    private Command coralCollect(){
        return new SequentialCommandGroup(
                Commands.runOnce(()-> new LowerCoralElevator(coralElevator)),
                Commands.runOnce(()-> coralArmCommand.setNewTargetPosition(RobotMap.ARM_CORAL_ANGLE_B)),
                Commands.waitUntil(()-> coralArmCommand.didReachTargetPosition()),
                new ReleaseCoral(coralGripper));
    }

    private Command algaeCollect(){
        return new SequentialCommandGroup(
                Commands.runOnce(()-> new ExtendedAlgaeArm(algaeArm)), new CollectAlgae(algaeGripper));
    }

    private Command algaeOut(){
        return new SequentialCommandGroup(
                Commands.runOnce(()-> new RetractAlgaeArm(algaeArm)), new ReleaseAlgae(algaeGripper));
    }
}
